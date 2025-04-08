/*******************************************************************************
  Copyright(c) 2024 Jérémie Klein. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.

 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#include "ups_nuts.h"
#include "indicom.h"
#include "connectionplugins/connectiontcp.h"

#include <memory>
#include <cstring>
#include <unistd.h>
#include <termios.h>

static std::unique_ptr<UPSSafety> upsSafety(new UPSSafety());

UPSSafety::UPSSafety() : INDI::DefaultDevice(), INDI::WeatherInterface(this)
{
    setVersion(1, 0);

    tcpConnection = new Connection::TCP(this);
    tcpConnection->setDefaultHost("localhost");
    tcpConnection->setDefaultPort(3493);
    registerConnection(tcpConnection);
}

bool UPSSafety::initProperties()
{
    INDI::DefaultDevice::initProperties();
    
    // Initialize weather interface
    WeatherInterface::initProperties("Status", "Parameters");
    
    // Add battery parameters
    addParameter("BATTERY_CHARGE", "Charge", 20, 100, 10, false);  // Warning at 10% from limits
    addParameter("BATTERY_VOLTAGE", "Voltage", 11, 13, 5, false);  // Warning at 5% from limits
    
    // Set battery charge as critical parameter
    setCriticalParameter("BATTERY_CHARGE");

    // Simulation Mode
    SimulationSP[0].fill("SIMULATION_OFF", "Off", ISS_ON);
    SimulationSP[1].fill("SIMULATION_ON", "On", ISS_OFF);
    SimulationSP.setDeviceName(getDeviceName());
    SimulationSP.setName("SIMULATION");
    SimulationSP.setLabel("Simulation");
    SimulationSP.setGroupName("Simulation");
    SimulationSP.setPermission(IP_RW);
    SimulationSP.setRule(ISR_1OFMANY);
    SimulationSP.setState(IPS_IDLE);

    // Simulation Settings
    SimulationNP[0].fill("SIM_BATTERY_CHARGE", "Battery Charge (%)", "%.1f", 0, 100, 1, 100);
    SimulationNP[1].fill("SIM_BATTERY_VOLTAGE", "Battery Voltage (V)", "%.1f", 0, 15, 0.1, 12);
    SimulationNP.setDeviceName(getDeviceName());
    SimulationNP.setName("SIMULATION_SETTINGS");
    SimulationNP.setLabel("Simulation Settings");
    SimulationNP.setGroupName("Simulation");
    SimulationNP.setPermission(IP_RW);
    SimulationNP.setState(IPS_IDLE);

    // UPS Connection Settings
    UPSConnectionTP[0].fill("UPS_NAME", "UPS Name", "ups");
    UPSConnectionTP.setDeviceName(getDeviceName());
    UPSConnectionTP.setName("UPS_CONNECTION");
    UPSConnectionTP.setLabel("UPS Connection");
    UPSConnectionTP.setGroupName("Connection");
    UPSConnectionTP.setPermission(IP_RW);
    UPSConnectionTP.setState(IPS_IDLE);

    // UPS Status
    StatusLP[0].fill("STATUS_ONLINE", "Online", IPS_IDLE);
    StatusLP[1].fill("STATUS_CHARGING", "Charging", IPS_IDLE);
    StatusLP[2].fill("STATUS_DISCHARGING", "Discharging", IPS_IDLE);
    StatusLP[3].fill("STATUS_LOW_BATTERY", "Low Battery", IPS_IDLE);
    StatusLP.setDeviceName(getDeviceName());
    StatusLP.setName("UPS_STATUS");
    StatusLP.setLabel("Status");
    StatusLP.setGroupName("Main Control");
    StatusLP.setState(IPS_IDLE);

    // UPS Info
    InfoTP[0].fill("INFO_STATUS", "Status", "");
    InfoTP.setDeviceName(getDeviceName());
    InfoTP.setName("UPS_INFO");
    InfoTP.setLabel("UPS Info");
    InfoTP.setGroupName("Main Control");
    InfoTP.setPermission(IP_RO);
    InfoTP.setState(IPS_IDLE);

    addDebugControl();
    setDriverInterface(WEATHER_INTERFACE);

    // Let the base driver handle the connection properties
    tcpConnection->setDefaultHost("localhost");
    tcpConnection->setDefaultPort(3493);

    return true;
}

bool UPSSafety::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(StatusLP);
        defineProperty(InfoTP);
    }
    else
    {
        deleteProperty(StatusLP);
        deleteProperty(InfoTP);
    }

    // Always define these properties
    defineProperty(UPSConnectionTP);
    defineProperty(SimulationSP);
    defineProperty(SimulationNP);

    return true;
}

const char *UPSSafety::getDefaultName()
{
    return "UPS Safety";
}

bool UPSSafety::Connect()
{
    LOG_INFO("Attempting to connect to UPS Safety...");
    
    bool rc = INDI::DefaultDevice::Connect();
    if (!rc)
    {
        LOG_ERROR("Base connection failed");
        return false;
    }

    if (!isSimulation())
    {
        // Ensure we have a valid file descriptor
        PortFD = tcpConnection->getPortFD();
        if (PortFD == -1)
        {
            LOG_ERROR("Failed to get TCP file descriptor");
            return false;
        }

        if (!Handshake())
        {
            LOG_ERROR("Handshake failed");
            return false;
        }
    }
    else
    {
        LOG_INFO("Simulation mode is active");
    }

    LOG_INFO("UPS Safety connected successfully.");
    SetTimer(getCurrentPollingPeriod());
    
    return true;
}

bool UPSSafety::Disconnect()
{
    if (!isSimulation())
    {
        PortFD = -1;
    }
    
    bool rc = INDI::DefaultDevice::Disconnect();
    if (rc)
    {
        LOG_INFO("UPS Safety disconnected successfully.");
    }
    return rc;
}

void UPSSafety::TimerHit()
{
    if (!isConnected())
        return;

    updateWeather();
    SetTimer(getCurrentPollingPeriod());
}

bool UPSSafety::sendCommand(const std::string &command, std::vector<std::string> &response)
{
    if (isSimulation())
    {
        // Simulate responses based on command
        if (command.find("LIST UPS") != std::string::npos)
        {
            response.push_back("ups \"Simulated UPS 1\"");
            return true;
        }
        else if (command.find("battery.charge") != std::string::npos)
        {
            response.push_back("VAR ups battery.charge \"" + std::to_string(SimulationNP[0].getValue()) + "\"");
            return true;
        }
        else if (command.find("battery.voltage") != std::string::npos)
        {
            response.push_back("VAR ups battery.voltage \"" + std::to_string(SimulationNP[1].getValue()) + "\"");
            return true;
        }
        else if (command.find("ups.status") != std::string::npos)
        {
            response.push_back("VAR ups ups.status \"OL CHRG\"");
            return true;
        }
        return true;
    }

    if (PortFD == -1)
    {
        LOG_ERROR("Device not connected");
        return false;
    }

    char buffer[BUFFER_SIZE];
    std::string cmd = command + "\n";
    
    LOGF_DEBUG("CMD: %s", cmd.c_str());

    // Send command
    int nbytes_written = write(PortFD, cmd.c_str(), cmd.length());
    if (nbytes_written < 0)
    {
        LOGF_ERROR("Write error: %s", strerror(errno));
        return false;
    }

    // Short delay
    usleep(100000);

    // Read response
    response.clear();
    int nbytes = read(PortFD, buffer, BUFFER_SIZE - 1);
    
    if (nbytes <= 0)
    {
        LOGF_DEBUG("No response (nbytes=%d)", nbytes);
        return false;
    }

    // Process response
    buffer[nbytes] = '\0';
    std::string responseStr(buffer, nbytes);
    LOGF_DEBUG("RES: %s", responseStr.c_str());

    // Split into lines
    size_t pos = 0;
    std::string line;
    while ((pos = responseStr.find('\n')) != std::string::npos)
    {
        line = responseStr.substr(0, pos);
        if (!line.empty())
        {
            response.push_back(line);
        }
        responseStr.erase(0, pos + 1);
    }
    if (!responseStr.empty())
    {
        response.push_back(responseStr);
    }

    return !response.empty();
}

bool UPSSafety::Handshake()
{
    LOG_INFO("Establishing connection with NUT server...");

    if (isSimulation())
    {
        LOG_INFO("Simulation mode active - skipping handshake");
        return true;
    }

    // Verify PortFD again to be sure
    PortFD = tcpConnection->getPortFD();
    if (PortFD == -1)
    {
        LOG_ERROR("Failed to get TCP file descriptor");
        return false;
    }

    LOGF_DEBUG("Connection established, PortFD = %d", PortFD);

    // Simple communication test
    std::vector<std::string> response;
    if (!sendCommand("LIST UPS", response))
    {
        LOG_ERROR("Error retrieving UPS list");
        return false;
    }

    // Display found UPS devices
    LOG_INFO("Available UPS devices:");
    for (const auto &line : response)
    {
        LOGF_INFO("%s", line.c_str());
    }

    if (response.empty())
    {
        LOG_ERROR("No UPS found");
        return false;
    }
    
    // Check if the specified UPS exists
    std::string upsName = UPSConnectionTP[0].getText();
    bool upsFound = false;
    
    for (const auto &line : response)
    {
        std::string upsPart = upsName + " ";
        if (line.find(upsPart) == 0)
        {
            upsFound = true;
            LOGF_INFO("UPS '%s' found", upsName.c_str());
            break;
        }
    }
    
    if (!upsFound)
    {
        LOGF_ERROR("UPS '%s' not found. Please verify the name and try again.", upsName.c_str());
        return false;
    }

    LOG_INFO("UPS Safety is online. Retrieving parameters...");
    return true;
}

template<typename T>
bool UPSSafety::extractNUTValue(const std::vector<std::string> &response, const std::string &varName, T &value, std::string &strValue)
{
    bool found = false;
    for (const auto &line : response)
    {
        LOGF_DEBUG("Processing %s line: '%s'", varName.c_str(), line.c_str());
        if (line.find("VAR") == 0)
        {
            char valueStr[256];
            if (sscanf(line.c_str(), "VAR %*s %*s \"%[^\"]\"", valueStr) == 1)
            {
                LOGF_DEBUG("Extracted %s value: '%s'", varName.c_str(), valueStr);
                strValue = valueStr;
                try
                {
                    if constexpr(std::is_same_v<T, std::string>)
                    {
                        value = valueStr;
                    }
                    else
                    {
                        value = std::stod(valueStr);
                    }
                    found = true;
                }
                catch (const std::exception &e)
                {
                    LOGF_ERROR("Error parsing %s: %s", varName.c_str(), e.what());
                }
            }
            else
            {
                LOGF_DEBUG("Failed to extract %s value using sscanf", varName.c_str());
            }
        }
        else if (line.find("ERR") == 0)
        {
            LOGF_DEBUG("Variable %s not supported: %s", varName.c_str(), line.c_str());
        }
    }
    return found;
}

void UPSSafety::processUPSState(const char *status)
{
    StatusLP[0].setState(IPS_IDLE);
    StatusLP[1].setState(IPS_IDLE);
    StatusLP[2].setState(IPS_IDLE);
    StatusLP[3].setState(IPS_IDLE);

    if (strstr(status, "OL") != nullptr)
        StatusLP[0].setState(IPS_OK);
    if (strstr(status, "CHRG") != nullptr)
        StatusLP[1].setState(IPS_OK);
    if (strstr(status, "DISCHRG") != nullptr)
        StatusLP[2].setState(IPS_BUSY);
    if (strstr(status, "LB") != nullptr)
        StatusLP[3].setState(IPS_ALERT);

    StatusLP.apply();
}

bool UPSSafety::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (UPSConnectionTP.isNameMatch(name))
        {
            UPSConnectionTP.update(texts, names, n);
            UPSConnectionTP.setState(IPS_OK);
            UPSConnectionTP.apply();
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool UPSSafety::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (SimulationNP.isNameMatch(name))
        {
            SimulationNP.update(values, names, n);
            SimulationNP.setState(IPS_OK);
            SimulationNP.apply();
            return true;
        }
        return WeatherInterface::processNumber(dev, name, values, names, n);
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool UPSSafety::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (SimulationSP.isNameMatch(name))
        {
            SimulationSP.update(states, names, n);
            SimulationSP.setState(IPS_OK);
            SimulationSP.apply();
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool UPSSafety::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    WeatherInterface::saveConfigItems(fp);
    UPSConnectionTP.save(fp);
    SimulationSP.save(fp);
    SimulationNP.save(fp);
    return true;
}

IPState UPSSafety::updateWeather()
{
    if (!isConnected())
        return IPS_ALERT;

    // Handle simulation mode
    if (isSimulation())
    {
        setParameterValue("BATTERY_CHARGE", SimulationNP[0].getValue());
        setParameterValue("BATTERY_VOLTAGE", SimulationNP[1].getValue());
        
        // Set simulated status
        StatusLP[0].setState(IPS_OK);
        StatusLP[1].setState(IPS_IDLE);
        StatusLP[2].setState(IPS_IDLE);
        StatusLP[3].setState(IPS_IDLE);
        StatusLP.apply();

        InfoTP[0].setText("SIMULATION");
        InfoTP.setState(IPS_OK);
        InfoTP.apply();

        return IPS_OK;
    }

    std::vector<std::string> response;
    std::string upsName = UPSConnectionTP[0].getText();
    std::string strValue;
    bool success = true;

    // Update battery charge
    std::string command = "GET VAR " + upsName + " battery.charge";
    double charge;
    if (sendCommand(command, response) && 
        extractNUTValue(response, "battery.charge", charge, strValue))
    {
        setParameterValue("BATTERY_CHARGE", charge);
    }
    else
    {
        success = false;
    }

    // Update battery voltage
    command = "GET VAR " + upsName + " battery.voltage";
    double voltage;
    if (sendCommand(command, response) && 
        extractNUTValue(response, "battery.voltage", voltage, strValue))
    {
        setParameterValue("BATTERY_VOLTAGE", voltage);
    }
    else
    {
        success = false;
    }

    // Update UPS status
    command = "GET VAR " + upsName + " ups.status";
    std::string status;
    if (sendCommand(command, response) && 
        extractNUTValue(response, "ups.status", status, strValue))
    {
        processUPSState(status.c_str());
        InfoTP[0].setText(status.c_str());
        InfoTP.setState(IPS_OK);
    }
    else
    {
        InfoTP[0].setText("N/A");
        InfoTP.setState(IPS_ALERT);
    }
    InfoTP.apply();

    return success ? IPS_OK : IPS_ALERT;
} 