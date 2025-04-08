/*******************************************************************************
  Copyright(c) 2025 Jérémie Klein. All rights reserved.

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

#include <memory>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include "ups_safety.h"
#include "indicom.h"

static std::unique_ptr<UPSSafety> upsSafety(new UPSSafety());

UPSSafety::UPSSafety() : INDI::WeatherInterface(this)
{
    setVersion(1, 0);

    tcpConnection = new Connection::TCP(this);
    tcpConnection->setDefaultHost("localhost");
    tcpConnection->setDefaultPort(3493);
    registerConnection(tcpConnection);
}

UPSSafety::~UPSSafety()
{
}

bool UPSSafety::initProperties()
{
    INDI::DefaultDevice::initProperties();
    
    // Initialize WeatherInterface
    INDI::WeatherInterface::initProperties("SAFETY_GROUP", MAIN_CONTROL_TAB);
    
    // Add UPS Safety parameter
    addParameter("UPS_SAFETY", "UPS Safety", 0, 1, 0);  // min=0 (safe), max=1 (warning), 2=danger
    setCriticalParameter("UPS_SAFETY");
    
    // UPS Connection Settings - moved to CONNECTION_TAB to be available before connecting
    UPSConnectionTP[UPS_NAME].fill("UPS_NAME", "UPS Name", "ups");
    UPSConnectionTP.fill(getDeviceName(), "UPS_CONNECTION", "UPS Connection", CONNECTION_TAB, IP_RW, 60, IPS_IDLE);

    // UPS Status
    StatusLP[STATUS_ONLINE].fill("STATUS_ONLINE", "Online", IPS_IDLE);
    StatusLP[STATUS_CHARGING].fill("STATUS_CHARGING", "Charging", IPS_IDLE);
    StatusLP[STATUS_DISCHARGING].fill("STATUS_DISCHARGING", "Discharging", IPS_IDLE);
    StatusLP[STATUS_LOW_BATTERY].fill("STATUS_LOW_BATTERY", "Low Battery", IPS_IDLE);
    StatusLP.fill(getDeviceName(), "UPS_STATUS", "Status", MAIN_CONTROL_TAB, IPS_IDLE);

    // Battery Info
    BatteryNP[BATTERY_CHARGE].fill("BATTERY_CHARGE", "Charge (%)", "%3.0f", 0, 100, 1, 0);
    BatteryNP[BATTERY_VOLTAGE].fill("BATTERY_VOLTAGE", "Voltage (V)", "%4.1f", 0, 999, 0.1, 0);
    BatteryNP.fill(getDeviceName(), "UPS_BATTERY", "Battery", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // UPS Info - ne garder que le statut
    InfoTP[INFO_STATUS].fill("INFO_STATUS", "Status", "");
    InfoTP.fill(getDeviceName(), "UPS_INFO", "UPS Info", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    // Safety Triggers
    SafetyTriggersNP[TRIGGER_LOW_BATTERY].fill("TRIGGER_LOW_BATTERY", "Low Battery (%)", "%3.0f", 0, 100, 1, 20);
    SafetyTriggersNP[TRIGGER_CRITICAL_BATTERY].fill("TRIGGER_CRITICAL_BATTERY", "Critical Battery (%)", "%3.0f", 0, 100, 1, 5);
    SafetyTriggersNP.fill(getDeviceName(), "SAFETY_TRIGGERS", "Safety Triggers", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // Safety Actions
    SafetyActionsTP[ACTION_LOW_BATTERY].fill("ACTION_LOW_BATTERY", "Low Battery Action", "WARN");
    SafetyActionsTP[ACTION_CRITICAL_BATTERY].fill("ACTION_CRITICAL_BATTERY", "Critical Battery Action", "PARK");
    SafetyActionsTP.fill(getDeviceName(), "SAFETY_ACTIONS", "Safety Actions", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // Add Safety Parameter that other devices can monitor (like Weather's critical parameters)
    SafetyParameterNP[PARAM_SAFETY_STATUS].fill("SAFETY_STATUS", "Safety Status", "%.0f", 0, 2, 1, 0);
    SafetyParameterNP.fill(getDeviceName(), "SAFETY_PARAMETER", "Safety", MAIN_CONTROL_TAB, IP_RO, 60, IPS_OK);

    addDebugControl();
    setDriverInterface(AUX_INTERFACE);

    tcpConnection->setDefaultPort(3493);
    
    // Define UPS Connection property here so it's available before connecting
    defineProperty(UPSConnectionTP);

    return true;
}

bool UPSSafety::sendCommand(const std::string &command, std::vector<std::string> &response)
{
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

    // Lire la réponse
    response.clear();
    int nbytes = read(PortFD, buffer, BUFFER_SIZE - 1);
    
    if (nbytes <= 0)
    {
        LOGF_DEBUG("No response (nbytes=%d)", nbytes);
        return false;
    }

    // Traiter la réponse
    buffer[nbytes] = '\0';
    std::string responseStr(buffer, nbytes);
    LOGF_DEBUG("RES: %s", responseStr.c_str());

    // Séparer en lignes
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
    std::string upsName = UPSConnectionTP[UPS_NAME].getText();
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

bool UPSSafety::updateProperties()
{
    INDI::DefaultDevice::updateProperties();
    INDI::WeatherInterface::updateProperties();

    if (isConnected())
    {
        // UPSConnectionTP is already defined, no need to define it again
        defineProperty(StatusLP);
        defineProperty(BatteryNP);
        defineProperty(InfoTP);
        defineProperty(SafetyTriggersNP);
        defineProperty(SafetyActionsTP);
        defineProperty(SafetyParameterNP);
    }
    else
    {
        // Don't delete UPSConnectionTP, we want it to remain available
        deleteProperty(StatusLP);
        deleteProperty(BatteryNP);
        deleteProperty(InfoTP);
        deleteProperty(SafetyTriggersNP);
        deleteProperty(SafetyActionsTP);
        deleteProperty(SafetyParameterNP);
    }

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

    // Ensure we have a valid file descriptor
    PortFD = tcpConnection->getPortFD();
    if (PortFD == -1)
    {
        LOG_ERROR("Failed to get TCP file descriptor");
        return false;
    }

    LOG_INFO("UPS Safety connected successfully.");
    updateUPSInfo();
    SetTimer(getCurrentPollingPeriod());
    
    return true;
}

bool UPSSafety::Disconnect()
{
    PortFD = -1;
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

    updateUPSStatus();
    checkSafetyTriggers();

    SetTimer(getCurrentPollingPeriod());
}

// Nouvelle méthode helper pour extraire une valeur d'une réponse NUT
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

// Version simplifiée de updateUPSStatus utilisant la nouvelle méthode helper
void UPSSafety::updateUPSStatus()
{
    if (!isConnected())
        return;

    std::vector<std::string> response;
    std::string upsName = UPSConnectionTP[UPS_NAME].getText();
    bool allOK = true;
    std::string strValue;

    // Structure pour définir les variables à récupérer
    struct {
        const char *command;
        double &value;
        double factor;
    } queries[] = {
        {"battery.charge", BatteryNP[BATTERY_CHARGE].value, 1.0},
        {"battery.voltage", BatteryNP[BATTERY_VOLTAGE].value, 1.0}
    };

    // Récupérer toutes les valeurs numériques
    for (const auto &query : queries)
    {
        std::string command = "GET VAR " + upsName + " " + query.command;
        double tmpValue;
        
        if (sendCommand(command, response) && 
            extractNUTValue(response, query.command, tmpValue, strValue))
        {
            query.value = tmpValue * query.factor;
        }
        else
        {
            allOK = false;
        }
    }

    BatteryNP.setState(allOK ? IPS_OK : IPS_ALERT);
    BatteryNP.apply();

    // Get UPS status
    std::string command = "GET VAR " + upsName + " ups.status";
    std::string status;
    
    if (sendCommand(command, response) && 
        extractNUTValue(response, "ups.status", status, strValue))
    {
        processUPSState(status.c_str());
        IUSaveText(&InfoTP[INFO_STATUS], status.c_str());
        InfoTP.setState(IPS_OK);
    }
    else
    {
        IUSaveText(&InfoTP[INFO_STATUS], "N/A");
        InfoTP.setState(IPS_ALERT);
    }
    InfoTP.apply();
}

// Version simplifiée de updateUPSInfo utilisant la nouvelle méthode helper
void UPSSafety::updateUPSInfo()
{
    if (!isConnected())
        return;

    std::vector<std::string> response;
    std::string upsName = UPSConnectionTP[UPS_NAME].getText();
    std::string strValue;

    // Get UPS status only
    std::string command = "GET VAR " + upsName + " ups.status";
    std::string value;
    
    if (sendCommand(command, response) && 
        extractNUTValue(response, "ups.status", value, strValue))
    {
        IUSaveText(&InfoTP[INFO_STATUS], value.c_str());
        processUPSState(value.c_str());
        InfoTP.setState(IPS_OK);
    }
    else
    {
        IUSaveText(&InfoTP[INFO_STATUS], "N/A");
        InfoTP.setState(IPS_ALERT);
    }

    InfoTP.apply();
}

void UPSSafety::processUPSState(const char *status)
{
    StatusLP[STATUS_ONLINE].setState(IPS_IDLE);
    StatusLP[STATUS_CHARGING].setState(IPS_IDLE);
    StatusLP[STATUS_DISCHARGING].setState(IPS_IDLE);
    StatusLP[STATUS_LOW_BATTERY].setState(IPS_IDLE);

    if (strstr(status, "OL") != nullptr)
        StatusLP[STATUS_ONLINE].setState(IPS_OK);
    if (strstr(status, "CHRG") != nullptr)
        StatusLP[STATUS_CHARGING].setState(IPS_OK);
    if (strstr(status, "DISCHRG") != nullptr)
        StatusLP[STATUS_DISCHARGING].setState(IPS_BUSY);
    if (strstr(status, "LB") != nullptr)
        StatusLP[STATUS_LOW_BATTERY].setState(IPS_ALERT);

    StatusLP.apply();
}

// Implement WeatherInterface method
IPState UPSSafety::updateWeather()
{
    if (!isConnected())
        return IPS_ALERT;
    
    // Update is handled directly by TimerHit
    return IPS_OK;
}

void UPSSafety::checkSafetyTriggers()
{
    double batteryCharge = BatteryNP[BATTERY_CHARGE].getValue();
    double lowTrigger = SafetyTriggersNP[TRIGGER_LOW_BATTERY].getValue();
    double criticalTrigger = SafetyTriggersNP[TRIGGER_CRITICAL_BATTERY].getValue();

    // Update safety parameter
    if (batteryCharge <= criticalTrigger)
    {
        // Critical level (2 = Danger)
        SafetyParameterNP[PARAM_SAFETY_STATUS].setValue(2);
        SafetyParameterNP.setState(IPS_ALERT);
        setParameterValue("UPS_SAFETY", 2);

        if (!m_CriticalBatteryTriggered)
        {
            m_CriticalBatteryTriggered = true;
            LOGF_ERROR("CRITICAL BATTERY LEVEL: %.1f%% - Executing action: %s", 
                      batteryCharge, SafetyActionsTP[ACTION_CRITICAL_BATTERY].getText());
            IDMessage(getDeviceName(), "CRITICAL BATTERY LEVEL: %.1f%%", batteryCharge);
        }
    }
    else if (batteryCharge <= lowTrigger)
    {
        // Warning level (1 = Warning)
        SafetyParameterNP[PARAM_SAFETY_STATUS].setValue(1);
        SafetyParameterNP.setState(IPS_BUSY);
        setParameterValue("UPS_SAFETY", 1);

        if (!m_LowBatteryTriggered)
        {
            m_LowBatteryTriggered = true;
            LOGF_WARN("LOW BATTERY LEVEL: %.1f%% - Executing action: %s", 
                     batteryCharge, SafetyActionsTP[ACTION_LOW_BATTERY].getText());
            IDMessage(getDeviceName(), "LOW BATTERY LEVEL: %.1f%%", batteryCharge);
        }
    }
    else
    {
        // Safe level (0 = Safe)
        SafetyParameterNP[PARAM_SAFETY_STATUS].setValue(0);
        SafetyParameterNP.setState(IPS_OK);
        setParameterValue("UPS_SAFETY", 0);
        m_LowBatteryTriggered = false;
        m_CriticalBatteryTriggered = false;
    }

    // Update the safety parameter
    SafetyParameterNP.apply();
    
    // Also ensure weather parameters are updated
    critialParametersLP.apply();
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
        else if (SafetyActionsTP.isNameMatch(name))
        {
            SafetyActionsTP.update(texts, names, n);
            SafetyActionsTP.setState(IPS_OK);
            SafetyActionsTP.apply();
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool UPSSafety::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (SafetyTriggersNP.isNameMatch(name))
        {
            // Validate that critical level is lower than low level
            double newLowLevel = -1;
            double newCriticalLevel = -1;

            for (int i = 0; i < n; i++)
            {
                if (strcmp(names[i], SafetyTriggersNP[TRIGGER_LOW_BATTERY].getName()) == 0)
                    newLowLevel = values[i];
                else if (strcmp(names[i], SafetyTriggersNP[TRIGGER_CRITICAL_BATTERY].getName()) == 0)
                    newCriticalLevel = values[i];
            }

            if (newLowLevel != -1 && newCriticalLevel != -1 && newCriticalLevel >= newLowLevel)
            {
                LOG_ERROR("Critical battery level must be lower than low battery level");
                SafetyTriggersNP.setState(IPS_ALERT);
                SafetyTriggersNP.apply();
                return false;
            }

            SafetyTriggersNP.update(values, names, n);
            SafetyTriggersNP.setState(IPS_OK);
            SafetyTriggersNP.apply();
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool UPSSafety::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    INDI::WeatherInterface::saveConfigItems(fp);

    UPSConnectionTP.save(fp);
    SafetyTriggersNP.save(fp);
    SafetyActionsTP.save(fp);
    SafetyParameterNP.save(fp);

    return true;
} 