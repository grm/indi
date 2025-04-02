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

#include "discord_notifier.h"

#include "indicom.h"
#include "indilogger.h"
#include "connectionplugins/connectionserial.h"
#include "indidevapi.h"  // Pour IDGetDevice
#include "baseclient.h"

#include <memory>
#include <cstring>
#include <unistd.h>
#include <sys/stat.h>

// Include the CURL library
#include <curl/curl.h>

// Standard properties
static constexpr const char *DISCORD_TAB = "Discord";
static constexpr const char *WEBHOOK_PROPERTY = "DISCORD_WEBHOOK";
static constexpr const char *BOT_CONFIG_PROPERTY = "BOT_CONFIG";
static constexpr const char *MESSAGE_FORMAT_PROPERTY = "MESSAGE_FORMAT";
static constexpr const char *SNOOP_DEVICES_PROPERTY = "SNOOP_DEVICES";
static constexpr const char *NOTIFICATION_PROPERTY = "NOTIFICATION";
static constexpr const char *TEST_NOTIFICATION_PROPERTY = "TEST_NOTIFICATION";

// Callback function for CURL
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string *)userp)->append((char *)contents, size * nmemb);
    return size * nmemb;
}

std::unique_ptr<DiscordNotifier> discordNotifier(new DiscordNotifier());

DiscordNotifier::DiscordNotifier()
{
    setVersion(1, 0);
    curl_global_init(CURL_GLOBAL_ALL);
    messageCountLastMinute = 0;
    lastMessageTime = std::chrono::system_clock::now();
}

DiscordNotifier::~DiscordNotifier()
{
    curl_global_cleanup();
}

bool DiscordNotifier::initProperties()
{
    INDI::DefaultDevice::initProperties();

    // Discord webhook URL
    IUFillText(&WebhookT[0], "URL", "Webhook URL", "");
    IUFillTextVector(&WebhookTP, WebhookT, 1, getDeviceName(), WEBHOOK_PROPERTY, "Discord Webhook",
                     DISCORD_TAB, IP_RW, 60, IPS_IDLE);

    // Discord bot configuration
    IUFillText(&BotConfigT[0], "BOT_NAME", "Bot Name", "INDI Notifier");
    IUFillText(&BotConfigT[1], "AVATAR_URL", "Avatar URL", "");
    IUFillTextVector(&BotConfigTP, BotConfigT, 2, getDeviceName(), BOT_CONFIG_PROPERTY, "Bot Configuration",
                     DISCORD_TAB, IP_RW, 60, IPS_IDLE);

    // Message format configuration
    IUFillText(&MessageFormatT[0], "TEMPLATE", "Message Template", "**{device}**: {property}.{element} = {value}");
    IUFillTextVector(&MessageFormatTP, MessageFormatT, 1, getDeviceName(), MESSAGE_FORMAT_PROPERTY, "Message Format",
                     DISCORD_TAB, IP_RW, 60, IPS_IDLE);

    // Snooped devices configuration
    IUFillText(&SnoopDevicesT[DEVICE_MOUNT], "MOUNT", "Mount", "");
    IUFillText(&SnoopDevicesT[DEVICE_CCD], "CCD", "Camera", "");
    IUFillText(&SnoopDevicesT[DEVICE_FOCUSER], "FOCUSER", "Focuser", "");
    IUFillText(&SnoopDevicesT[DEVICE_FILTERWHEEL], "FILTERWHEEL", "Filter Wheel", "");
    IUFillText(&SnoopDevicesT[DEVICE_DOME], "DOME", "Dome", "");
    IUFillText(&SnoopDevicesT[DEVICE_WEATHER], "WEATHER", "Weather", "");
    IUFillTextVector(&SnoopDevicesTP, SnoopDevicesT, DEVICE_COUNT, getDeviceName(), SNOOP_DEVICES_PROPERTY, "Devices to Monitor",
                     DISCORD_TAB, IP_RW, 60, IPS_IDLE);

    // Mount properties
    IUFillSwitch(&MountPropertiesS[MOUNT_CONNECTION], "MOUNT_CONNECTION", "Connection", ISS_ON);
    IUFillSwitch(&MountPropertiesS[MOUNT_INFO], "MOUNT_INFO", "Mount Info", ISS_ON);
    IUFillSwitch(&MountPropertiesS[MOUNT_PARK], "MOUNT_PARK", "Park Status", ISS_ON);
    IUFillSwitch(&MountPropertiesS[MOUNT_COORDS], "MOUNT_COORDS", "Coordinates", ISS_ON);
    IUFillSwitch(&MountPropertiesS[MOUNT_SLEW], "MOUNT_SLEW", "Slew Rate", ISS_ON);
    IUFillSwitch(&MountPropertiesS[MOUNT_MOTION_NS], "MOUNT_MOTION_NS", "Motion N/S", ISS_ON);
    IUFillSwitch(&MountPropertiesS[MOUNT_MOTION_WE], "MOUNT_MOTION_WE", "Motion E/W", ISS_ON);
    IUFillSwitchVector(&MountPropertiesSP, MountPropertiesS, MOUNT_PROPERTY_COUNT, getDeviceName(), "MOUNT_PROPS", "Mount Properties",
                       "Mount", IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // CCD properties
    IUFillSwitch(&CCDPropertiesS[CCD_CONNECTION], "CCD_CONNECTION", "Connection", ISS_ON);
    IUFillSwitch(&CCDPropertiesS[CCD_EXPOSURE], "CCD_EXPOSURE", "Exposure", ISS_ON);
    IUFillSwitch(&CCDPropertiesS[CCD_TEMP], "CCD_TEMP", "Temperature", ISS_ON);
    IUFillSwitch(&CCDPropertiesS[CCD_FRAME], "CCD_FRAME", "Frame", ISS_ON);
    IUFillSwitch(&CCDPropertiesS[CCD_BINNING], "CCD_BINNING", "Binning", ISS_ON);
    IUFillSwitchVector(&CCDPropertiesSP, CCDPropertiesS, CCD_PROPERTY_COUNT, getDeviceName(), "CCD_PROPS", "Camera Properties",
                       "Camera", IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // Focuser properties
    IUFillSwitch(&FocuserPropertiesS[FOCUSER_CONNECTION], "FOCUSER_CONNECTION", "Connection", ISS_ON);
    IUFillSwitch(&FocuserPropertiesS[FOCUSER_MOTION], "FOCUSER_MOTION", "Motion", ISS_ON);
    IUFillSwitch(&FocuserPropertiesS[FOCUSER_POSITION], "FOCUSER_POSITION", "Position", ISS_ON);
    IUFillSwitch(&FocuserPropertiesS[FOCUSER_TEMP], "FOCUSER_TEMP", "Temperature", ISS_ON);
    IUFillSwitchVector(&FocuserPropertiesSP, FocuserPropertiesS, FOCUSER_PROPERTY_COUNT, getDeviceName(), "FOCUSER_PROPS", "Focuser Properties",
                       "Focuser", IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // Filter Wheel properties
    IUFillSwitch(&FilterWheelPropertiesS[FILTER_CONNECTION], "FILTER_CONNECTION", "Connection", ISS_ON);
    IUFillSwitch(&FilterWheelPropertiesS[FILTER_SLOT], "FILTER_SLOT", "Filter Slot", ISS_ON);
    IUFillSwitchVector(&FilterWheelPropertiesSP, FilterWheelPropertiesS, FILTER_PROPERTY_COUNT, getDeviceName(), "FILTER_PROPS", "Filter Properties",
                       "Filter Wheel", IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // Dome properties
    IUFillSwitch(&DomePropertiesS[DOME_CONNECTION], "DOME_CONNECTION", "Connection", ISS_ON);
    IUFillSwitch(&DomePropertiesS[DOME_MOTION], "DOME_MOTION", "Motion", ISS_ON);
    IUFillSwitch(&DomePropertiesS[DOME_SHUTTER], "DOME_SHUTTER", "Shutter", ISS_ON);
    IUFillSwitchVector(&DomePropertiesSP, DomePropertiesS, DOME_PROPERTY_COUNT, getDeviceName(), "DOME_PROPS", "Dome Properties",
                       "Dome", IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // Weather properties
    IUFillSwitch(&WeatherPropertiesS[WEATHER_CONNECTION], "WEATHER_CONNECTION", "Connection", ISS_ON);
    IUFillSwitch(&WeatherPropertiesS[WEATHER_STATUS], "WEATHER_STATUS", "Status", ISS_ON);
    IUFillSwitchVector(&WeatherPropertiesSP, WeatherPropertiesS, WEATHER_PROPERTY_COUNT, getDeviceName(), "WEATHER_PROPS", "Weather Properties",
                       "Weather", IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // Enable/disable notifications
    IUFillSwitch(&NotificationS[0], "NOTIFICATION_ON", "On", ISS_OFF);
    IUFillSwitch(&NotificationS[1], "NOTIFICATION_OFF", "Off", ISS_ON);
    IUFillSwitchVector(&NotificationSP, NotificationS, 2, getDeviceName(), NOTIFICATION_PROPERTY, "Notifications",
                       DISCORD_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Test notification
    IUFillSwitch(&TestNotificationS[0], "SEND_TEST", "Send Test", ISS_OFF);
    IUFillSwitchVector(&TestNotificationSP, TestNotificationS, 1, getDeviceName(), TEST_NOTIFICATION_PROPERTY, "Test",
                       DISCORD_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);
    
    // Device-specific notification switches
    IUFillSwitch(&MountNotifyS[0], "MOUNT_NOTIFY_ON", "Enable", ISS_ON);
    IUFillSwitch(&MountNotifyS[1], "MOUNT_NOTIFY_OFF", "Disable", ISS_OFF);
    IUFillSwitchVector(&MountNotifySP, MountNotifyS, 2, getDeviceName(), "MOUNT_NOTIFICATIONS", "Mount Notifications",
                       "Mount", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&CCDNotifyS[0], "CCD_NOTIFY_ON", "Enable", ISS_ON);
    IUFillSwitch(&CCDNotifyS[1], "CCD_NOTIFY_OFF", "Disable", ISS_OFF);
    IUFillSwitchVector(&CCDNotifySP, CCDNotifyS, 2, getDeviceName(), "CCD_NOTIFICATIONS", "Camera Notifications",
                       "Camera", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&FocuserNotifyS[0], "FOCUSER_NOTIFY_ON", "Enable", ISS_ON);
    IUFillSwitch(&FocuserNotifyS[1], "FOCUSER_NOTIFY_OFF", "Disable", ISS_OFF);
    IUFillSwitchVector(&FocuserNotifySP, FocuserNotifyS, 2, getDeviceName(), "FOCUSER_NOTIFICATIONS", "Focuser Notifications",
                       "Focuser", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&FilterWheelNotifyS[0], "FILTERWHEEL_NOTIFY_ON", "Enable", ISS_ON);
    IUFillSwitch(&FilterWheelNotifyS[1], "FILTERWHEEL_NOTIFY_OFF", "Disable", ISS_OFF);
    IUFillSwitchVector(&FilterWheelNotifySP, FilterWheelNotifyS, 2, getDeviceName(), "FILTERWHEEL_NOTIFICATIONS", "Filter Wheel Notifications",
                       "Filter Wheel", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&DomeNotifyS[0], "DOME_NOTIFY_ON", "Enable", ISS_ON);
    IUFillSwitch(&DomeNotifyS[1], "DOME_NOTIFY_OFF", "Disable", ISS_OFF);
    IUFillSwitchVector(&DomeNotifySP, DomeNotifyS, 2, getDeviceName(), "DOME_NOTIFICATIONS", "Dome Notifications",
                       "Dome", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&WeatherNotifyS[0], "WEATHER_NOTIFY_ON", "Enable", ISS_ON);
    IUFillSwitch(&WeatherNotifyS[1], "WEATHER_NOTIFY_OFF", "Disable", ISS_OFF);
    IUFillSwitchVector(&WeatherNotifySP, WeatherNotifyS, 2, getDeviceName(), "WEATHER_NOTIFICATIONS", "Weather Notifications",
                       "Weather", IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Message batching configuration
    IUFillNumber(&BatchConfigN[0], "BATCH_SIZE", "Messages per batch", "%.0f", 1, 30, 1, 10);
    IUFillNumber(&BatchConfigN[1], "BATCH_TIMEOUT", "Batch timeout (seconds)", "%.0f", 1, 60, 1, 10);
    IUFillNumberVector(&BatchConfigNP, BatchConfigN, 2, getDeviceName(), "BATCH_CONFIG", "Message Batching",
                      DISCORD_TAB, IP_RW, 60, IPS_IDLE);

    setDriverInterface(AUX_INTERFACE);
    addDebugControl();

    return true;
}

bool DiscordNotifier::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    // Ces propriétés doivent toujours être accessibles
    defineProperty(&WebhookTP);
    defineProperty(&BotConfigTP);
    defineProperty(&MessageFormatTP);
    defineProperty(&SnoopDevicesTP);

    if (isConnected())
    {
        // Propriétés spécifiques à l'état connecté
        defineProperty(&MountPropertiesSP);
        defineProperty(&CCDPropertiesSP);
        defineProperty(&FocuserPropertiesSP);
        defineProperty(&FilterWheelPropertiesSP);
        defineProperty(&DomePropertiesSP);
        defineProperty(&WeatherPropertiesSP);
        defineProperty(&NotificationSP);
        defineProperty(&TestNotificationSP);
        defineProperty(&MountNotifySP);
        defineProperty(&CCDNotifySP);
        defineProperty(&FocuserNotifySP);
        defineProperty(&FilterWheelNotifySP);
        defineProperty(&DomeNotifySP);
        defineProperty(&WeatherNotifySP);
        defineProperty(&BatchConfigNP);
        
        // Configure le snooping pour les périphériques configurés
        setupSnooping();
    }
    else
    {
        // Supprimer uniquement les propriétés spécifiques à l'état connecté
        deleteProperty(MountPropertiesSP.name);
        deleteProperty(CCDPropertiesSP.name);
        deleteProperty(FocuserPropertiesSP.name);
        deleteProperty(FilterWheelPropertiesSP.name);
        deleteProperty(DomePropertiesSP.name);
        deleteProperty(WeatherPropertiesSP.name);
        deleteProperty(NotificationSP.name);
        deleteProperty(TestNotificationSP.name);
        deleteProperty(MountNotifySP.name);
        deleteProperty(CCDNotifySP.name);
        deleteProperty(FocuserNotifySP.name);
        deleteProperty(FilterWheelNotifySP.name);
        deleteProperty(DomeNotifySP.name);
        deleteProperty(WeatherNotifySP.name);
        deleteProperty(BatchConfigNP.name);
    }

    return true;
}

void DiscordNotifier::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);
    
    // Define configuration properties at startup
    defineProperty(&WebhookTP);
    defineProperty(&BotConfigTP);
    defineProperty(&MessageFormatTP);
    defineProperty(&SnoopDevicesTP);
    
    // Load saved configuration
    loadConfig(true, WEBHOOK_PROPERTY);
    loadConfig(true, BOT_CONFIG_PROPERTY);
    loadConfig(true, MESSAGE_FORMAT_PROPERTY);
    loadConfig(true, SNOOP_DEVICES_PROPERTY);
    loadConfig(true, "MOUNT_PROPS");
    loadConfig(true, "CCD_PROPS");
    loadConfig(true, "FOCUSER_PROPS");
    loadConfig(true, "FILTER_PROPS");
    loadConfig(true, "DOME_PROPS");
    loadConfig(true, "WEATHER_PROPS");
    loadConfig(true, NOTIFICATION_PROPERTY);
    
    // Ne définir les propriétés de notification que si le périphérique est connecté
    if (isConnected())
    {
        defineProperty(&MountPropertiesSP);
        defineProperty(&CCDPropertiesSP);
        defineProperty(&FocuserPropertiesSP);
        defineProperty(&FilterWheelPropertiesSP);
        defineProperty(&DomePropertiesSP);
        defineProperty(&WeatherPropertiesSP);
        defineProperty(&NotificationSP);
        defineProperty(&MountNotifySP);
        defineProperty(&CCDNotifySP);
        defineProperty(&FocuserNotifySP);
        defineProperty(&FilterWheelNotifySP);
        defineProperty(&DomeNotifySP);
        defineProperty(&WeatherNotifySP);
    }
}

const char *DiscordNotifier::getDefaultName()
{
    return "Discord Notifier";
}

bool DiscordNotifier::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Handle Discord webhook URL
        if (strcmp(name, WEBHOOK_PROPERTY) == 0)
        {
            IUUpdateText(&WebhookTP, texts, names, n);
            WebhookTP.s = IPS_OK;
            IDSetText(&WebhookTP, nullptr);
            return true;
        }
        
        // Handle bot configuration
        if (strcmp(name, BOT_CONFIG_PROPERTY) == 0)
        {
            IUUpdateText(&BotConfigTP, texts, names, n);
            BotConfigTP.s = IPS_OK;
            IDSetText(&BotConfigTP, nullptr);
            return true;
        }
        
        // Handle message format
        if (strcmp(name, MESSAGE_FORMAT_PROPERTY) == 0)
        {
            IUUpdateText(&MessageFormatTP, texts, names, n);
            MessageFormatTP.s = IPS_OK;
            IDSetText(&MessageFormatTP, nullptr);
            return true;
        }

        // Handle snooped devices
        if (strcmp(name, SNOOP_DEVICES_PROPERTY) == 0)
        {
            IUUpdateText(&SnoopDevicesTP, texts, names, n);
            SnoopDevicesTP.s = IPS_OK;
            IDSetText(&SnoopDevicesTP, nullptr);
            
            // Reconfigure snooping if connected
            if (isConnected())
            {
                setupSnooping();
            }
            
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool DiscordNotifier::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Handle device-specific properties
        if (strcmp(name, MountPropertiesSP.name) == 0)
        {
            IUUpdateSwitch(&MountPropertiesSP, states, names, n);
            MountPropertiesSP.s = IPS_OK;
            IDSetSwitch(&MountPropertiesSP, nullptr);
            if (isConnected()) setupSnooping();
            return true;
        }
        
        if (strcmp(name, CCDPropertiesSP.name) == 0)
        {
            IUUpdateSwitch(&CCDPropertiesSP, states, names, n);
            CCDPropertiesSP.s = IPS_OK;
            IDSetSwitch(&CCDPropertiesSP, nullptr);
            if (isConnected()) setupSnooping();
            return true;
        }
        
        if (strcmp(name, FocuserPropertiesSP.name) == 0)
        {
            IUUpdateSwitch(&FocuserPropertiesSP, states, names, n);
            FocuserPropertiesSP.s = IPS_OK;
            IDSetSwitch(&FocuserPropertiesSP, nullptr);
            if (isConnected()) setupSnooping();
            return true;
        }
        
        if (strcmp(name, FilterWheelPropertiesSP.name) == 0)
        {
            IUUpdateSwitch(&FilterWheelPropertiesSP, states, names, n);
            FilterWheelPropertiesSP.s = IPS_OK;
            IDSetSwitch(&FilterWheelPropertiesSP, nullptr);
            if (isConnected()) setupSnooping();
            return true;
        }
        
        if (strcmp(name, DomePropertiesSP.name) == 0)
        {
            IUUpdateSwitch(&DomePropertiesSP, states, names, n);
            DomePropertiesSP.s = IPS_OK;
            IDSetSwitch(&DomePropertiesSP, nullptr);
            if (isConnected()) setupSnooping();
            return true;
        }
        
        if (strcmp(name, WeatherPropertiesSP.name) == 0)
        {
            IUUpdateSwitch(&WeatherPropertiesSP, states, names, n);
            WeatherPropertiesSP.s = IPS_OK;
            IDSetSwitch(&WeatherPropertiesSP, nullptr);
            if (isConnected()) setupSnooping();
            return true;
        }
        
        // Handle notification switch
        if (strcmp(name, NOTIFICATION_PROPERTY) == 0)
        {
            IUUpdateSwitch(&NotificationSP, states, names, n);
            NotificationSP.s = IPS_OK;
            
            if (NotificationS[0].s == ISS_ON)
            {
                // Éviter d'envoyer le message si c'est juste une réinitialisation des propriétés
                if (!isConnected())
                {
                    NotificationS[0].s = ISS_OFF;
                    NotificationS[1].s = ISS_ON;
                    NotificationSP.s = IPS_IDLE;
                    IDSetSwitch(&NotificationSP, nullptr);
                    return true;
                }
                sendDiscordMessage("Discord notifications enabled for INDI");
            }
            
            IDSetSwitch(&NotificationSP, nullptr);
            return true;
        }
        
        // Handle test notification
        if (strcmp(name, TEST_NOTIFICATION_PROPERTY) == 0)
        {
            IUUpdateSwitch(&TestNotificationSP, states, names, n);
            
            if (TestNotificationS[0].s == ISS_ON)
            {
                TestNotificationS[0].s = ISS_OFF;
                TestNotificationSP.s = IPS_OK;
                IDSetSwitch(&TestNotificationSP, nullptr);
                
                if (sendDiscordMessage("Test notification from INDI Discord Notifier"))
                {
                    TestNotificationSP.s = IPS_OK;
                    IDSetSwitch(&TestNotificationSP, nullptr);
                }
                else
                {
                    TestNotificationSP.s = IPS_ALERT;
                    IDSetSwitch(&TestNotificationSP, nullptr);
                }
            }
            
            return true;
        }

        // Handle device notification switches
        if (strcmp(name, MountNotifySP.name) == 0)
        {
            IUUpdateSwitch(&MountNotifySP, states, names, n);
            MountNotifySP.s = IPS_OK;
            IDSetSwitch(&MountNotifySP, nullptr);
            return true;
        }

        if (strcmp(name, CCDNotifySP.name) == 0)
        {
            IUUpdateSwitch(&CCDNotifySP, states, names, n);
            CCDNotifySP.s = IPS_OK;
            IDSetSwitch(&CCDNotifySP, nullptr);
            return true;
        }

        if (strcmp(name, FocuserNotifySP.name) == 0)
        {
            IUUpdateSwitch(&FocuserNotifySP, states, names, n);
            FocuserNotifySP.s = IPS_OK;
            IDSetSwitch(&FocuserNotifySP, nullptr);
            return true;
        }

        if (strcmp(name, FilterWheelNotifySP.name) == 0)
        {
            IUUpdateSwitch(&FilterWheelNotifySP, states, names, n);
            FilterWheelNotifySP.s = IPS_OK;
            IDSetSwitch(&FilterWheelNotifySP, nullptr);
            return true;
        }

        if (strcmp(name, DomeNotifySP.name) == 0)
        {
            IUUpdateSwitch(&DomeNotifySP, states, names, n);
            DomeNotifySP.s = IPS_OK;
            IDSetSwitch(&DomeNotifySP, nullptr);
            return true;
        }

        if (strcmp(name, WeatherNotifySP.name) == 0)
        {
            IUUpdateSwitch(&WeatherNotifySP, states, names, n);
            WeatherNotifySP.s = IPS_OK;
            IDSetSwitch(&WeatherNotifySP, nullptr);
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool DiscordNotifier::ISSnoopDevice(XMLEle *root)
{
    if (NotificationS[0].s != ISS_ON)
    {
        return false;
    }

    const char *device = findXMLAttValu(root, "device");
    const char *propName = findXMLAttValu(root, "name");

    if (!device || !propName)
    {
        LOG_ERROR("Invalid snoop data received - missing device or property name");
        return false;
    }

    for (XMLEle *ep = nextXMLEle(root, 1); ep != nullptr; ep = nextXMLEle(root, 0))
    {
        const char *elemName = findXMLAttValu(ep, "name");
        const char *elemValue = pcdataXMLEle(ep);

        if (elemName && elemValue)
        {
            processDeviceEvent(device, propName, elemName, elemValue);
        }
    }

    return true;
}

void DiscordNotifier::setupSnooping()
{
    // Au lieu d'écouter tous les périphériques, on n'écoute que ceux qui sont configurés
    if (strlen(SnoopDevicesT[DEVICE_MOUNT].text) > 0)
        setupSnoopingForDevice(SnoopDevicesT[DEVICE_MOUNT].text, "TYPE_MOUNT");
    
    if (strlen(SnoopDevicesT[DEVICE_CCD].text) > 0)
        setupSnoopingForDevice(SnoopDevicesT[DEVICE_CCD].text, "TYPE_CCD");
    
    if (strlen(SnoopDevicesT[DEVICE_FOCUSER].text) > 0)
        setupSnoopingForDevice(SnoopDevicesT[DEVICE_FOCUSER].text, "TYPE_FOCUSER");
    
    if (strlen(SnoopDevicesT[DEVICE_FILTERWHEEL].text) > 0)
        setupSnoopingForDevice(SnoopDevicesT[DEVICE_FILTERWHEEL].text, "TYPE_FILTERWHEEL");
    
    if (strlen(SnoopDevicesT[DEVICE_DOME].text) > 0)
        setupSnoopingForDevice(SnoopDevicesT[DEVICE_DOME].text, "TYPE_DOME");
    
    if (strlen(SnoopDevicesT[DEVICE_WEATHER].text) > 0)
        setupSnoopingForDevice(SnoopDevicesT[DEVICE_WEATHER].text, "TYPE_WEATHER");
}

bool DiscordNotifier::sendDiscordMessage(const std::string &message)
{
    return queueDiscordMessage(message);
}

bool DiscordNotifier::queueDiscordMessage(const std::string &message)
{
    std::lock_guard<std::mutex> lock(mutex);
    
    messageQueue.emplace(message);
    
    SetTimer(100);
    
    return true;
}

void DiscordNotifier::TimerHit()
{
    if (!isConnected())
        return;

    processMessageQueue();
    SetTimer(100); // Check every 100ms
}

bool DiscordNotifier::shouldSendBatch()
{
    auto now = std::chrono::system_clock::now();
    auto timeoutSeconds = static_cast<int>(BatchConfigN[1].value);
    auto batchSize = static_cast<int>(BatchConfigN[0].value);
    
    // Send if we have enough messages
    if (messageQueue.size() >= static_cast<size_t>(batchSize))
        return true;
        
    // Send if enough time has passed since last batch
    if (!messageQueue.empty() && 
        std::chrono::duration_cast<std::chrono::seconds>(now - lastBatchTime).count() >= timeoutSeconds)
        return true;
        
    return false;
}

void DiscordNotifier::processMessageQueue()
{
    std::lock_guard<std::mutex> lock(mutex);
    
    auto now = std::chrono::system_clock::now();
    
    // Reset message counter every minute
    if (std::chrono::duration_cast<std::chrono::minutes>(now - lastMessageTime).count() >= 1)
    {
        messageCountLastMinute = 0;
        lastMessageTime = now;
    }
    
    // Check if we should send a batch
    if (!shouldSendBatch())
        return;
        
    // Check if we're under rate limit
    if (messageCountLastMinute >= MAX_MESSAGES_PER_MINUTE)
        return;
        
    // Process up to batchSize messages
    auto batchSize = static_cast<size_t>(BatchConfigN[0].value);
    std::vector<std::string> messages;
    
    while (!messageQueue.empty() && messages.size() < batchSize && 
           messageCountLastMinute < MAX_MESSAGES_PER_MINUTE)
    {
        messages.push_back(messageQueue.front().message);
        messageQueue.pop();
    }
    
    if (!messages.empty())
    {
        // Combine messages
        std::string combinedMessage;
        for (size_t i = 0; i < messages.size(); ++i)
        {
            combinedMessage += messages[i];
            if (i < messages.size() - 1)
                combinedMessage += "\n";
        }
        
        // Send to Discord
        CURL *curl = curl_easy_init();
        if (curl)
        {
            std::string json = "{\"content\":\"" + combinedMessage + "\"";
            if (strlen(BotConfigT[0].text) > 0)
                json += ",\"username\":\"" + std::string(BotConfigT[0].text) + "\"";
            if (strlen(BotConfigT[1].text) > 0)
                json += ",\"avatar_url\":\"" + std::string(BotConfigT[1].text) + "\"";
            json += "}";
            
            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            
            curl_easy_setopt(curl, CURLOPT_URL, WebhookT[0].text);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);
            curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 1L);
            
            std::string response;
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            
            CURLcode res = curl_easy_perform(curl);
            if (res == CURLE_OK)
            {
                long http_code = 0;
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
                
                if (http_code == 204)
                {
                    messageCountLastMinute += messages.size();
                    lastMessageTime = now;
                    lastBatchTime = now;
                    LOGF_DEBUG("Successfully sent batch of %d messages to Discord", (int)messages.size());
                }
                else if (http_code == 429)
                {
                    // Requeue messages on rate limit
                    for (const auto& msg : messages)
                        messageQueue.emplace(msg);
                    LOG_WARN("Discord rate limit reached, messages requeued");
                }
            }
            else
            {
                // Requeue messages on error
                for (const auto& msg : messages)
                    messageQueue.emplace(msg);
                LOGF_WARN("Failed to send messages: %s", curl_easy_strerror(res));
            }
            
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }
}

void DiscordNotifier::processDeviceEvent(const char *device, const char *property, const char *element, const char *value)
{
    LOGF_DEBUG("Processing device event - Device: %s, Property: %s, Element: %s, Value: %s", 
               device, property, element, value);

    if (!device || !property || !element || !value)
    {
        LOGF_ERROR("Invalid event data - Device: %s, Property: %s, Element: %s, Value: %s",
                   device ? device : "null",
                   property ? property : "null",
                   element ? element : "null",
                   value ? value : "null");
        return;
    }

    std::string messageTemplate = MessageFormatT[0].text;
    LOGF_DEBUG("Using message template: %s", messageTemplate.c_str());

    std::string message = messageTemplate;
    size_t pos;

    while ((pos = message.find("{device}")) != std::string::npos)
        message.replace(pos, 8, device);

    while ((pos = message.find("{property}")) != std::string::npos)
        message.replace(pos, 10, property);

    while ((pos = message.find("{element}")) != std::string::npos)
        message.replace(pos, 9, element);

    while ((pos = message.find("{value}")) != std::string::npos)
        message.replace(pos, 7, value);

    LOGF_DEBUG("Formatted message: %s", message.c_str());

    if (!sendDiscordMessage(message))
    {
        LOGF_ERROR("Failed to send Discord message for event - Device: %s, Property: %s",
                   device, property);
    }
}

bool DiscordNotifier::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    
    IUSaveConfigText(fp, &WebhookTP);
    IUSaveConfigText(fp, &BotConfigTP);
    IUSaveConfigText(fp, &MessageFormatTP);
    IUSaveConfigText(fp, &SnoopDevicesTP);
    IUSaveConfigSwitch(fp, &MountPropertiesSP);
    IUSaveConfigSwitch(fp, &CCDPropertiesSP);
    IUSaveConfigSwitch(fp, &FocuserPropertiesSP);
    IUSaveConfigSwitch(fp, &FilterWheelPropertiesSP);
    IUSaveConfigSwitch(fp, &DomePropertiesSP);
    IUSaveConfigSwitch(fp, &WeatherPropertiesSP);
    IUSaveConfigSwitch(fp, &NotificationSP);
    IUSaveConfigSwitch(fp, &MountNotifySP);
    IUSaveConfigSwitch(fp, &CCDNotifySP);
    IUSaveConfigSwitch(fp, &FocuserNotifySP);
    IUSaveConfigSwitch(fp, &FilterWheelNotifySP);
    IUSaveConfigSwitch(fp, &DomeNotifySP);
    IUSaveConfigSwitch(fp, &WeatherNotifySP);
    IUSaveConfigNumber(fp, &BatchConfigNP);
    
    return true;
}

bool DiscordNotifier::Connect()
{
    LOG_INFO("Connecting Discord Notifier...");
    
    // Vérifier la configuration du webhook avant de se connecter
    if (strlen(WebhookT[0].text) == 0)
    {
        LOG_ERROR("Cannot connect: Webhook URL is not configured");
        return false;
    }

    // Configurer le snooping uniquement pour les périphériques configurés
    setupSnooping();
    
    // Envoyer le message de connexion de manière asynchrone
    queueDiscordMessage("Discord Notifier connected");
    
    return true;
}

bool DiscordNotifier::Disconnect()
{
    LOG_INFO("Discord Notifier disconnected");
    return true;
}

void DiscordNotifier::setupSnoopingForDevice(const char* deviceName, const char* deviceType)
{
    if (!deviceName || strlen(deviceName) == 0)
        return;

    LOGF_INFO("Setting up snooping for device: %s (Type: %s)", deviceName, deviceType);

    if (strcmp(deviceType, "TYPE_MOUNT") == 0)
    {
        if (MountPropertiesS[MOUNT_CONNECTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "CONNECTION");
        if (MountPropertiesS[MOUNT_INFO].s == ISS_ON)
            IDSnoopDevice(deviceName, "TELESCOPE_INFO");
        if (MountPropertiesS[MOUNT_PARK].s == ISS_ON)
            IDSnoopDevice(deviceName, "TELESCOPE_PARK");
        if (MountPropertiesS[MOUNT_COORDS].s == ISS_ON) {
            IDSnoopDevice(deviceName, "EQUATORIAL_EOD_COORD");
            IDSnoopDevice(deviceName, "EQUATORIAL_COORD");
        }
        if (MountPropertiesS[MOUNT_SLEW].s == ISS_ON)
            IDSnoopDevice(deviceName, "TELESCOPE_SLEW_RATE");
        if (MountPropertiesS[MOUNT_MOTION_NS].s == ISS_ON)
            IDSnoopDevice(deviceName, "TELESCOPE_MOTION_NS");
        if (MountPropertiesS[MOUNT_MOTION_WE].s == ISS_ON)
            IDSnoopDevice(deviceName, "TELESCOPE_MOTION_WE");
    }
    else if (strcmp(deviceType, "TYPE_CCD") == 0)
    {
        if (CCDPropertiesS[CCD_CONNECTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "CONNECTION");
        if (CCDPropertiesS[CCD_EXPOSURE].s == ISS_ON)
            IDSnoopDevice(deviceName, "CCD_EXPOSURE");
        if (CCDPropertiesS[CCD_TEMP].s == ISS_ON)
            IDSnoopDevice(deviceName, "CCD_TEMPERATURE");
        if (CCDPropertiesS[CCD_FRAME].s == ISS_ON)
            IDSnoopDevice(deviceName, "CCD_FRAME");
        if (CCDPropertiesS[CCD_BINNING].s == ISS_ON)
            IDSnoopDevice(deviceName, "CCD_BINNING");
    }
    else if (strcmp(deviceType, "TYPE_FOCUSER") == 0)
    {
        if (FocuserPropertiesS[FOCUSER_CONNECTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "CONNECTION");
        if (FocuserPropertiesS[FOCUSER_MOTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "FOCUS_MOTION");
        if (FocuserPropertiesS[FOCUSER_POSITION].s == ISS_ON) {
            IDSnoopDevice(deviceName, "ABS_FOCUS_POSITION");
            IDSnoopDevice(deviceName, "REL_FOCUS_POSITION");
        }
        if (FocuserPropertiesS[FOCUSER_TEMP].s == ISS_ON)
            IDSnoopDevice(deviceName, "FOCUS_TEMPERATURE");
    }
    else if (strcmp(deviceType, "TYPE_FILTERWHEEL") == 0)
    {
        if (FilterWheelPropertiesS[FILTER_CONNECTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "CONNECTION");
        if (FilterWheelPropertiesS[FILTER_SLOT].s == ISS_ON) {
            IDSnoopDevice(deviceName, "FILTER_SLOT");
            IDSnoopDevice(deviceName, "FILTER_NAME");
        }
    }
    else if (strcmp(deviceType, "TYPE_DOME") == 0)
    {
        if (DomePropertiesS[DOME_CONNECTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "CONNECTION");
        if (DomePropertiesS[DOME_MOTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "DOME_MOTION");
        if (DomePropertiesS[DOME_SHUTTER].s == ISS_ON) {
            IDSnoopDevice(deviceName, "DOME_SHUTTER");
            IDSnoopDevice(deviceName, "DOME_PARK");
        }
    }
    else if (strcmp(deviceType, "TYPE_WEATHER") == 0)
    {
        if (WeatherPropertiesS[WEATHER_CONNECTION].s == ISS_ON)
            IDSnoopDevice(deviceName, "CONNECTION");
        if (WeatherPropertiesS[WEATHER_STATUS].s == ISS_ON) {
            IDSnoopDevice(deviceName, "WEATHER_STATUS");
            IDSnoopDevice(deviceName, "WEATHER_PARAMETERS");
        }
    }
}

bool DiscordNotifier::areNotificationsEnabled(const char* deviceType)
{
    if (!deviceType)
        return false;

    if (strcmp(deviceType, "TYPE_MOUNT") == 0)
        return MountNotifyS[0].s == ISS_ON;
    else if (strcmp(deviceType, "TYPE_CCD") == 0)
        return CCDNotifyS[0].s == ISS_ON;
    else if (strcmp(deviceType, "TYPE_FOCUSER") == 0)
        return FocuserNotifyS[0].s == ISS_ON;
    else if (strcmp(deviceType, "TYPE_FILTERWHEEL") == 0)
        return FilterWheelNotifyS[0].s == ISS_ON;
    else if (strcmp(deviceType, "TYPE_DOME") == 0)
        return DomeNotifyS[0].s == ISS_ON;
    else if (strcmp(deviceType, "TYPE_WEATHER") == 0)
        return WeatherNotifyS[0].s == ISS_ON;

    return false;
}

// Initialization
extern "C" {
void ISInit()
{
    static int isInit = 0;

    if (isInit)
        return;

    isInit = 1;
    if (discordNotifier.get() == nullptr)
    {
        discordNotifier.reset(new DiscordNotifier());
    }
}

void ISGetProperties(const char *dev)
{
    ISInit();
    discordNotifier->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    ISInit();
    discordNotifier->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    ISInit();
    discordNotifier->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    ISInit();
    discordNotifier->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
    ISInit();
    discordNotifier->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

void ISSnoopDevice(XMLEle *root)
{
    ISInit();
    discordNotifier->ISSnoopDevice(root);
}
} 