/*******************************************************************************
  Copyright(c) 2022 Jérémie Klein. All rights reserved.

  Wanderer cover V3

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
*******************************************************************************/

#include "wanderer_cover.h"

#include "indicom.h"
#include "connectionplugins/connectionserial.h"

#include <cerrno>
#include <cstring>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>

// We declare an auto pointer to WandererCover.
static std::unique_ptr<WandererCover> wanderercover(new WandererCover());
#define PARK 1
#define DATA_BITS 8
#define FLAT_CMD 6
#define WANDERER_RESPONSE_SIZE 1
#define COMMAND_WAITING_TIME 120

#define CLOSE_COVER_COMMAND "1000\n"
#define HANDSHAKE_COMMAND "1500001\n"
#define OPEN_COVER_COMMAND "1001\n"
#define TURN_OFF_LIGHT_PANEL_COMMAND "9999\n"
#define COLLISION_DETECTION_OPERATION_MODE_COMMAND "10000\n"

WandererCover::WandererCover() : LightBoxInterface(this, true)
{
    setVersion(1, 0);
}

bool WandererCover::initProperties()
{
    INDI::DefaultDevice::initProperties();

    // Status
    IUFillText(&StatusT[0], "Cover", "Cover", nullptr);
    IUFillText(&StatusT[1], "Light", "Light", nullptr);
    IUFillTextVector(&StatusTP, StatusT, 2, getDeviceName(), "Status", "Status", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // Firmware version
    IUFillText(&FirmwareT[0], "Version", "Version", "pre 20220920");
    IUFillTextVector(&FirmwareTP, FirmwareT, 1, getDeviceName(), "Firmware", "Firmware", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    initDustCapProperties(getDeviceName(), MAIN_CONTROL_TAB);
    initLightBoxProperties(getDeviceName(), MAIN_CONTROL_TAB);

    LightIntensityN[0].min = 1;
    LightIntensityN[0].max = 255;
    LightIntensityN[0].step = 10;

    setDriverInterface(AUX_INTERFACE | LIGHTBOX_INTERFACE | DUSTCAP_INTERFACE);

    addAuxControls();

    serialConnection = new Connection::Serial(this);
    serialConnection->setDefaultBaudRate(Connection::Serial::B_19200);
    serialConnection->registerHandshake([&]()
                                        { return handshake(); });
    registerConnection(serialConnection);

    return true;
}

bool WandererCover::handshake()
{
    PortFD = serialConnection->getPortFD();
    tcflush(PortFD, TCIOFLUSH);
    int nbytes_read_name = 0,  nbytes_read_version = 0, nbytes_written = 0, rc = -1;
    char res_name[64] = {0};
    char res_version[64] = {0};


    LOGF_DEBUG("CMD <%s>", HANDSHAKE_COMMAND);
    if ((rc = tty_write_string(PortFD, HANDSHAKE_COMMAND, &nbytes_written)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Serial write error: %s", errorMessage);
        return false;
    }

    if ((rc = tty_read_section(PortFD, res_name, 'A', 5, &nbytes_read_name)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Device read error: %s", errorMessage);
        return false;
    }

    // We read version, prior to 20220920 it was not supported so some functionnality may not be available
    // But some are working so cnnection is successful
    if ((rc = tty_read_section(PortFD, res_version, 'A', 5, &nbytes_read_version)) != TTY_OK)
    { 
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        isVersionPriorTo20220920 = true;
        LOGF_INFO("Version not available: probably due to firmware version being pre 20220920. You should update the wanderer cover firmware to have all functionnalities", "");
    } else {
        // Some infos are only available in firmware ppre 20220920, such as firmware version, the cover status and the new operation mode
        // Firmware version
        res_version[nbytes_read_version - 1] = '\0';
        IUSaveText(&FirmwareT[0], res_version);
        IDSetText(&FirmwareTP, nullptr);
        // Cover status
        char res_cover_state[64] = {0};
        int nbytes_read_cover_state = 0;

        if ((rc = tty_read_section(PortFD, res_cover_state, 'A', 5, &nbytes_read_cover_state)) != TTY_OK)
        { 
                char errorMessage[MAXRBUF];
                tty_error_msg(rc, errorMessage, MAXRBUF);
                isVersionPriorTo20220920 = true;
                LOGF_ERROR("Device read error: %s", errorMessage);
        }
        updateCoverStatus(res_cover_state); 

        // Operation mode available
        char res_operation_mode[64] = {0};
        int nbytes_read_operation_mode = 0;
        IUFillSwitch(&DustCapMotorOperationModeS[COLLISION], "OPERATION_MODE_COLLISION", "Collision", ISS_OFF);
        IUFillSwitch(&DustCapMotorOperationModeS[LIMIT], "OPERATION_MODE_LIMIT_POSITION", "Limit", ISS_OFF);
        IUFillSwitchVector(&DustCapMotorOperationModeSP, DustCapMotorOperationModeS, 2, getDeviceName(), "OPERATION_MODE", "Operation Mode", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0,
                    IPS_IDLE);

        if ((rc = tty_read_section(PortFD, res_operation_mode, 'A', 5, &nbytes_read_operation_mode)) != TTY_OK)
        { 
                char errorMessage[MAXRBUF];
                tty_error_msg(rc, errorMessage, MAXRBUF);
                isVersionPriorTo20220920 = true;
                LOGF_ERROR("Device read error: %s", errorMessage);
        }

        res_operation_mode[nbytes_read_operation_mode - 1] = '\0';
        updateOperationMode(res_operation_mode); 
    }

    res_name[nbytes_read_name - 1] = '\0';

    if (!isVersionPriorTo20220920) {

    }

    LOGF_DEBUG("RES <%s>", res_name);
    LOGF_INFO("Handshake successful:%s", res_name);
    tcflush(PortFD, TCIOFLUSH);
    return true;
}

void WandererCover::updateCoverStatus(char* res) {
    if (strcmp(res,"0")==0) {
        setParkCapStatusAsClosed();
    } else if (strcmp(res,"1")==0) {
        setParkCapStatusAsOpen();
    } else if (strcmp(res,"255")==0) {
        //Unknown
    }
}

void WandererCover::updateOperationMode(char* res) {
    if (strcmp(res,"0")==0) {
        setButtonOperationMode(COLLISION);
    } else {
        numberOfStepsBeetweenOpenAndClose = sscanf(res, "%d", &numberOfStepsBeetweenOpenAndClose);
        setButtonOperationMode(LIMIT);
    }
}

void WandererCover::setButtonOperationMode(DustCapMotorOperationMode operation_mode) {
    if (operation_mode == COLLISION) {
        IUResetSwitch(&DustCapMotorOperationModeSP);
        DustCapMotorOperationModeS[COLLISION].s = ISS_ON;
        DustCapMotorOperationModeS[LIMIT].s = ISS_OFF;
        DustCapMotorOperationModeSP.s = IPS_OK;
        IDSetSwitch(&DustCapMotorOperationModeSP, nullptr);
    } else {
        IUResetSwitch(&DustCapMotorOperationModeSP);
        DustCapMotorOperationModeS[LIMIT].s = ISS_ON;
        DustCapMotorOperationModeS[COLLISION].s = ISS_OFF;
        DustCapMotorOperationModeSP.s = IPS_OK;
        IDSetSwitch(&DustCapMotorOperationModeSP, nullptr);    }
}


void WandererCover::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);

    // Get Light box properties
    isGetLightBoxProperties(dev);
}

bool WandererCover::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&ParkCapSP);
        defineProperty(&LightSP);
        defineProperty(&LightIntensityNP);
        defineProperty(&StatusTP);

        if (!isVersionPriorTo20220920) {
            defineProperty(&FirmwareTP);
        }

        updateLightBoxProperties();

        getStartupData();
    }
    else
    {
        deleteProperty(ParkCapSP.name);
        deleteProperty(LightSP.name);
        deleteProperty(LightIntensityNP.name);
        deleteProperty(StatusTP.name);

        if (! isVersionPriorTo20220920) {
            deleteProperty(FirmwareTP.name);
        }

        updateLightBoxProperties();
    }

    return true;
}

const char *WandererCover::getDefaultName()
{
    return "Wanderer Cover v3";
}

bool WandererCover::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (processLightBoxNumber(dev, name, values, names, n))
        return true;

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool WandererCover::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (processLightBoxText(dev, name, texts, names, n))
            return true;
    }

    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool WandererCover::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (processDustCapSwitch(dev, name, states, names, n))
            return true;

        if (processLightBoxSwitch(dev, name, states, names, n))
            return true;
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool WandererCover::processOperationModeSwitch(const char *dev, const char *name, ISState *states, char *names[],
                int n) 
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        // Operation Mode
        if (!strcmp(DustCapMotorOperationModeSP.name, name))
        {
            int prevIndex = IUFindOnSwitchIndex(&DustCapMotorOperationModeSP);
            IUUpdateSwitch(&DustCapMotorOperationModeSP, states, names, n);
            
            bool rc = switchOperationMode();

            DustCapMotorOperationModeSP.s = rc ? IPS_OK : IPS_ALERT;

            if (!rc)
            {
                IUResetSwitch(&DustCapMotorOperationModeSP);
                DustCapMotorOperationModeS[prevIndex].s = ISS_ON;
            }

            IDSetSwitch(&DustCapMotorOperationModeSP, nullptr); 

            return true;
        }
    }

    return false;
}

bool WandererCover::ISSnoopDevice(XMLEle *root)
{
    snoopLightBox(root);

    return INDI::DefaultDevice::ISSnoopDevice(root);
}

bool WandererCover::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);

    return saveLightBoxConfigItems(fp);
}

bool WandererCover::getStartupData()
{
    // Closing cover
    IUSaveText(&StatusT[0], "Closed");
    IUResetSwitch(&ParkCapSP);
    ParkCapS[0].s = ISS_ON;
    ParkCapSP.s = IPS_OK;
    LOG_INFO("Cover assumed as closed.");
    IDSetSwitch(&ParkCapSP, nullptr);

    // Switching off lamp
    IUSaveText(&StatusT[1], "Off");
    LightS[0].s = ISS_OFF;
    LightS[1].s = ISS_ON;
    IDSetSwitch(&LightSP, nullptr);
    LightIntensityN[0].value = 0;
    LOG_INFO("Light assumed as off.");
    IDSetNumber(&LightIntensityNP, nullptr);

    return true;
}

IPState WandererCover::ParkCap()
{
    char response[WANDERER_RESPONSE_SIZE];
    if (!sendCommand(CLOSE_COVER_COMMAND, response, false))
        return IPS_ALERT;

    setParkCapStatusAsClosed();
    return IPS_OK;
}

void WandererCover::setParkCapStatusAsClosed()
{
    IUSaveText(&StatusT[0], "Closed");
    IUResetSwitch(&ParkCapSP);
    ParkCapS[0].s = ISS_ON;
    ParkCapSP.s = IPS_OK;
    LOG_INFO("Cover closed.");
    IDSetSwitch(&ParkCapSP, nullptr);
}

IPState WandererCover::UnParkCap()
{
    char response[WANDERER_RESPONSE_SIZE];
    if (!sendCommand(OPEN_COVER_COMMAND, response, false))
        return IPS_ALERT;

    setParkCapStatusAsOpen();
    return IPS_OK;
}

void WandererCover::setParkCapStatusAsOpen()
{
    IUSaveText(&StatusT[0], "Open");
    IUResetSwitch(&ParkCapSP);
    ParkCapS[1].s = ISS_ON;
    ParkCapSP.s = IPS_OK;
    LOG_INFO("Cover open.");
    IDSetSwitch(&ParkCapSP, nullptr);
}

bool WandererCover::EnableLightBox(bool enable)
{
    if (ParkCapS[1].s == ISS_ON)
    {
        LOG_ERROR("Cannot control light while cap is unparked.");
        return false;
    }

    if (enable)
    {
        return SetLightBoxBrightness(255);
    }
    else
    {
        return switchOffLightBox();
    }

    return false;
}

bool WandererCover::switchOperationMode() {
    if (DustCapMotorOperationModeS[COLLISION].s == ISS_ON) {
        // TODO : manage open and close state
    } else {
        char response[WANDERER_RESPONSE_SIZE];
        if (!sendCommand(COLLISION_DETECTION_OPERATION_MODE_COMMAND, response, false))
            return false;
    }
    return true;
}

bool WandererCover::switchOffLightBox()
{
    char response[WANDERER_RESPONSE_SIZE];

    if (!sendCommand(TURN_OFF_LIGHT_PANEL_COMMAND, response, false))
        return false;

    IUSaveText(&StatusT[1], "Off");
    LightS[0].s = ISS_OFF;
    LightS[1].s = ISS_ON;
    LightIntensityN[0].value = 0;
    IDSetNumber(&LightIntensityNP, nullptr);
    IDSetSwitch(&LightSP, nullptr);
    return true;
}

bool WandererCover::SetLightBoxBrightness(uint16_t value)
{
    char response[WANDERER_RESPONSE_SIZE];
    char command[3] = {0};
    snprintf(command, 3, "%03d\n", value);
    if (!sendCommand(command, response, false))
        return false;

    LightIntensityN[0].value = value;
    IDSetNumber(&LightIntensityNP, nullptr);

    return true;
}

bool WandererCover::sendCommand(std::string command, char *response, bool waitForAnswer)
{
    int nbytes_read = 0, nbytes_written = 0, rc = -1;
    std::string command_termination = "\n";
    LOGF_DEBUG("CMD <%s>", command.c_str());
    if ((rc = tty_write_string(PortFD, (command + command_termination).c_str(), &nbytes_written)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Serial write error: %s", errorMessage);
        return false;
    }

    if (waitForAnswer && (rc = tty_read_section(PortFD, response, 'A', COMMAND_WAITING_TIME, &nbytes_read)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Device read error: %s", errorMessage);
        return false;
    }
    SetTimer(150);
    return true;
}
