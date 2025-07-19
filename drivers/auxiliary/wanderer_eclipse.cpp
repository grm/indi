/*******************************************************************************
  Copyright(c) 2025 Jérémie Klein. All rights reserved.

  Wanderer Eclipse

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

#include "wanderer_eclipse.h"
#include "indicom.h"
#include "connectionplugins/connectionserial.h"
#include <cstring>
#include <string>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <mutex>
#include <chrono>

static std::unique_ptr<WandererEclipse> wanderereclipse(new WandererEclipse());

WandererEclipse::WandererEclipse() : DustCapInterface(this)
{
    setVersion(1, 0);
}

const char *WandererEclipse::getDefaultName()
{
    return "WandererEclipse";
}

bool WandererEclipse::initProperties()
{
    INDI::DefaultDevice::initProperties();

    // Dust cap interface
    DI::initProperties(MAIN_CONTROL_TAB);
    setDriverInterface(AUX_INTERFACE | DUSTCAP_INTERFACE);
    addAuxControls();

    // Torque property
    TorqueSP[TORQUE_LOW].fill("TORQUE_LOW", "Low", ISS_OFF);
    TorqueSP[TORQUE_MEDIUM].fill("TORQUE_MEDIUM", "Medium", ISS_ON);
    TorqueSP[TORQUE_HIGH].fill("TORQUE_HIGH", "High", ISS_OFF);
    TorqueSP.fill(getDeviceName(), "TORQUE", "Motor Torque", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Firmware info
    FirmwareTP[FIRMWARE_VERSION].fill("FIRMWARE_VERSION", "Firmware Version", "Unknown");
    FirmwareTP.fill(getDeviceName(), "FIRMWARE_INFO", "Firmware", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // Voltage info
    VoltageNP[VOLTAGE].fill("VOLTAGE", "Voltage", "%.2f", 0.0, 30.0, 0.01, 0.0);
    VoltageNP.fill(getDeviceName(), "VOLTAGE_INFO", "Voltage", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    setDefaultPollingPeriod(2000);

    serialConnection = new Connection::Serial(this);
    serialConnection->setDefaultBaudRate(Connection::Serial::B_19200);
    serialConnection->registerHandshake([&]() {
        return requestStatus();
    });
    registerConnection(serialConnection);

    return true;
}

bool WandererEclipse::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        // Update firmware info
        char firmwareStr[16];
        snprintf(firmwareStr, sizeof(firmwareStr), "%d", firmware);
        FirmwareTP[FIRMWARE_VERSION].setText(firmwareStr);
        defineProperty(FirmwareTP);
        defineProperty(TorqueSP);
        defineProperty(VoltageNP);
    }
    else
    {
        deleteProperty(FirmwareTP);
        deleteProperty(TorqueSP);
        deleteProperty(VoltageNP);
    }

    DI::updateProperties();
    return true;
}

void WandererEclipse::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);
}

bool WandererEclipse::ISSnoopDevice(XMLEle *root)
{
    return INDI::DefaultDevice::ISSnoopDevice(root);
}

bool WandererEclipse::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()))
    {
        // Torque
        if (TorqueSP.isNameMatch(name))
        {
            int newTorque = -1;
            for (int i = 0; i < n; i++)
            {
                if (states[i] == ISS_ON)
                {
                    if (strcmp(names[i], "TORQUE_LOW") == 0)
                        newTorque = 0;
                    else if (strcmp(names[i], "TORQUE_MEDIUM") == 0)
                        newTorque = 1;
                    else if (strcmp(names[i], "TORQUE_HIGH") == 0)
                        newTorque = 2;
                }
            }
            if (newTorque != -1 && newTorque != torqueLevel)
            {
                // Send torque command: 3110 (low), 3160 (medium), 3190 (high)
                char cmd[8];
                if (newTorque == TORQUE_LOW)
                    snprintf(cmd, sizeof(cmd), "3110");
                else if (newTorque == TORQUE_MEDIUM)
                    snprintf(cmd, sizeof(cmd), "3160");
                else if (newTorque == TORQUE_HIGH)
                    snprintf(cmd, sizeof(cmd), "3190");
                
                if (sendCommand(cmd))
                {
                    torqueLevel = newTorque;
                    for (int i = 0; i < 3; i++)
                        TorqueSP[i].setState(i == newTorque ? ISS_ON : ISS_OFF);
                    TorqueSP.setState(IPS_OK);
                }
                else
                {
                    TorqueSP.setState(IPS_ALERT);
                }
                TorqueSP.apply();
                return true;
            }
        }
    }
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool WandererEclipse::toggleCover(bool open)
{
    char cmd[8] = {0};
    // 1001 = open, 1000 = close
    snprintf(cmd, sizeof(cmd), "100%d", open ? 1 : 0);
    return sendCommand(cmd);
}

IPState WandererEclipse::ParkCap()
{
    ParkCapSP.setState(IPS_BUSY);
    ParkCapSP.apply();
    if (toggleCover(false))
        return IPS_BUSY;
    ParkCapSP.setState(IPS_ALERT);
    ParkCapSP.apply();
    return IPS_ALERT;
}

IPState WandererEclipse::UnParkCap()
{
    ParkCapSP.setState(IPS_BUSY);
    ParkCapSP.apply();
    if (toggleCover(true))
        return IPS_BUSY;
    ParkCapSP.setState(IPS_ALERT);
    ParkCapSP.apply();
    return IPS_ALERT;
}

bool WandererEclipse::sendCommand(const std::string &command)
{
    std::lock_guard<std::timed_mutex> lock(serialPortMutex);
    int nbytes_written = 0, rc = -1;
    std::string command_termination = "\n";
    if ((rc = tty_write_string(PortFD, (command + command_termination).c_str(), &nbytes_written)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Serial write error: %s", errorMessage);
        return false;
    }
    return true;
}

bool WandererEclipse::requestStatus()
{
    std::lock_guard<std::timed_mutex> lock(serialPortMutex);
    PortFD = serialConnection->getPortFD();
    tcflush(PortFD, TCIOFLUSH);
    
    // Send status request command: 1500001
    if (!sendCommand("1500001"))
        return false;

    // Read device name first (should be "WandererEclipse")
    char deviceName[64] = {0};
    int nbytes_read = 0, rc = -1;
    
    if ((rc = tty_read_section(PortFD, deviceName, 'A', 5, &nbytes_read)) != TTY_OK)
    {
        if (rc == TTY_TIME_OUT)
        {
            LOG_DEBUG("Timeout reading device name, will try again later");
            return true;
        }
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Failed to read device name. Error: %s", errorMessage);
        return false;
    }
    deviceName[nbytes_read - 1] = '\0';
    LOGF_DEBUG("Device name: %s", deviceName);

    // Verify that this is indeed a WandererEclipse device
    if (strcmp(deviceName, "WandererEclipse") != 0)
    {
        LOGF_ERROR("Device identification failed. Expected 'WandererEclipse', got '%s'", deviceName);
        return false;
    }
    LOG_INFO("Successfully identified WandererEclipse device");

    // Read firmware version (Field 1)
    char firmwareVersion[64] = {0};
    if ((rc = tty_read_section(PortFD, firmwareVersion, 'A', 5, &nbytes_read)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Failed to read firmware version. Error: %s", errorMessage);
        return false;
    }
    firmwareVersion[nbytes_read - 1] = '\0';
    LOGF_DEBUG("Firmware version: %s", firmwareVersion);
    
    // Parse firmware version (YYYYMMDD format)
    int firmwareDate = 0;
    if (sscanf(firmwareVersion, "%d", &firmwareDate) == 1)
    {
        firmware = firmwareDate;
        char firmwareStr[16];
        snprintf(firmwareStr, sizeof(firmwareStr), "%d", firmware);
        FirmwareTP[FIRMWARE_VERSION].setText(firmwareStr);
        FirmwareTP.apply();
    }

    // Read motor torque (Field 2)
    char motorTorque[64] = {0};
    if ((rc = tty_read_section(PortFD, motorTorque, 'A', 5, &nbytes_read)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Failed to read motor torque. Error: %s", errorMessage);
        return false;
    }
    motorTorque[nbytes_read - 1] = '\0';
    LOGF_DEBUG("Motor torque: %s", motorTorque);
    
    // Parse motor torque and update UI
    int torqueValue = 0;
    if (sscanf(motorTorque, "%d", &torqueValue) == 1)
    {
        int newTorqueLevel = -1;
        if (torqueValue == 110)
            newTorqueLevel = TORQUE_LOW;
        else if (torqueValue == 160)
            newTorqueLevel = TORQUE_MEDIUM;
        else if (torqueValue == 190)
            newTorqueLevel = TORQUE_HIGH;
        
        if (newTorqueLevel != -1 && newTorqueLevel != torqueLevel)
        {
            torqueLevel = newTorqueLevel;
            for (int i = 0; i < 3; i++)
                TorqueSP[i].setState(i == newTorqueLevel ? ISS_ON : ISS_OFF);
            TorqueSP.setState(IPS_OK);
            TorqueSP.apply();
        }
    }

    // Read voltage (Field 3)
    char voltage[64] = {0};
    if ((rc = tty_read_section(PortFD, voltage, 'A', 5, &nbytes_read)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Failed to read voltage. Error: %s", errorMessage);
        return false;
    }
    voltage[nbytes_read - 1] = '\0';
    LOGF_DEBUG("Voltage: %s", voltage);
    
    // Parse voltage (xx.xx format)
    float voltageValue = 0.0f;
    if (sscanf(voltage, "%f", &voltageValue) == 1)
    {
        this->voltageValue = voltageValue;
        VoltageNP[VOLTAGE].setValue(voltageValue);
        VoltageNP.setState(IPS_OK);
        VoltageNP.apply();
        LOGF_INFO("Device voltage: %.2fV", voltageValue);
    }

    return true;
}

bool WandererEclipse::parseDeviceStatus(const char *data)
{
    // This method is now deprecated as parsing is done directly in requestStatus()
    // Keep for backward compatibility but log the data
    LOGF_DEBUG("Legacy parseDeviceStatus called with data: %s", data);
    return true;
}

void WandererEclipse::updateStatus(bool isOpen, int torqueLevel)
{
    // Update internal state and UI if needed
    this->isOpen = isOpen;
    this->torqueLevel = torqueLevel;
    // Update UI properties if needed
}

void WandererEclipse::TimerHit()
{
    if (!isConnected())
    {
        SetTimer(getPollingPeriod());
        return;
    }
    // Only request status on timer if needed (not real-time)
    requestStatus();
    SetTimer(getPollingPeriod());
}

bool WandererEclipse::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    TorqueSP.save(fp);
    VoltageNP.save(fp);
    return true;
} 