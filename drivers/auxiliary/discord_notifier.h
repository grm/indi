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

#pragma once

#include "defaultdevice.h"
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include <queue>
#include <chrono>

/**
 * @brief The DiscordNotifier class is an INDI device that snoops on other devices
 * and sends notifications to a Discord channel via webhooks.
 */
class DiscordNotifier : public INDI::DefaultDevice
{
    public:
        DiscordNotifier();
        virtual ~DiscordNotifier();

        // Énumérations pour les indices des propriétés
        enum MountPropertyIndex {
            MOUNT_CONNECTION = 0,
            MOUNT_INFO,
            MOUNT_PARK,
            MOUNT_COORDS,
            MOUNT_SLEW,
            MOUNT_MOTION_NS,
            MOUNT_MOTION_WE,
            MOUNT_PROPERTY_COUNT
        };

        enum CCDPropertyIndex {
            CCD_CONNECTION = 0,
            CCD_EXPOSURE,
            CCD_TEMP,
            CCD_FRAME,
            CCD_BINNING,
            CCD_PROPERTY_COUNT
        };

        enum FocuserPropertyIndex {
            FOCUSER_CONNECTION = 0,
            FOCUSER_MOTION,
            FOCUSER_POSITION,
            FOCUSER_TEMP,
            FOCUSER_PROPERTY_COUNT
        };

        enum FilterWheelPropertyIndex {
            FILTER_CONNECTION = 0,
            FILTER_SLOT,
            FILTER_PROPERTY_COUNT
        };

        enum DomePropertyIndex {
            DOME_CONNECTION = 0,
            DOME_MOTION,
            DOME_SHUTTER,
            DOME_PROPERTY_COUNT
        };

        enum WeatherPropertyIndex {
            WEATHER_CONNECTION = 0,
            WEATHER_STATUS,
            WEATHER_PROPERTY_COUNT
        };

        enum DeviceIndex {
            DEVICE_MOUNT = 0,
            DEVICE_CCD,
            DEVICE_FOCUSER,
            DEVICE_FILTERWHEEL,
            DEVICE_DOME,
            DEVICE_WEATHER,
            DEVICE_COUNT
        };

        // DefaultDevice functions
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual void ISGetProperties(const char *dev) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISSnoopDevice(XMLEle *root) override;
        virtual bool saveConfigItems(FILE *fp) override;
        
        // Connection methods
        virtual bool Connect() override;
        virtual bool Disconnect() override;

    protected:
        virtual const char *getDefaultName() override;

    private:
        // Helper functions
        void setupSnooping();
        void setupSnoopingForDevice(const char* deviceName, const char* deviceType);
        bool sendDiscordMessage(const std::string &message);
        void processDeviceEvent(const char *device, const char *property, const char *element, const char *value);
        
        // Properties
        // Discord webhook configuration
        IText WebhookT[1];
        ITextVectorProperty WebhookTP;
        
        // Discord bot configuration
        IText BotConfigT[2]; // [0] = Bot Name, [1] = Avatar URL
        ITextVectorProperty BotConfigTP;
        
        // Message format configuration
        IText MessageFormatT[1]; // Message template
        ITextVectorProperty MessageFormatTP;
        
        // Snooped devices configuration
        IText SnoopDevicesT[6]; // Mount, CCD, Focuser, Filter Wheel, Dome, Weather
        ITextVectorProperty SnoopDevicesTP;
        
        // Mount properties to snoop
        ISwitch MountPropertiesS[7];  // CONNECTION, TELESCOPE_INFO, TELESCOPE_PARK, etc.
        ISwitchVectorProperty MountPropertiesSP;
        
        // CCD properties to snoop
        ISwitch CCDPropertiesS[5];  // CONNECTION, CCD_EXPOSURE, CCD_TEMPERATURE, etc.
        ISwitchVectorProperty CCDPropertiesSP;
        
        // Focuser properties to snoop
        ISwitch FocuserPropertiesS[4];  // CONNECTION, FOCUS_MOTION, FOCUS_POSITION, etc.
        ISwitchVectorProperty FocuserPropertiesSP;
        
        // Filter Wheel properties to snoop
        ISwitch FilterWheelPropertiesS[2];  // CONNECTION, FILTER_SLOT, etc.
        ISwitchVectorProperty FilterWheelPropertiesSP;
        
        // Dome properties to snoop
        ISwitch DomePropertiesS[3];  // CONNECTION, DOME_MOTION, DOME_SHUTTER
        ISwitchVectorProperty DomePropertiesSP;
        
        // Weather properties to snoop
        ISwitch WeatherPropertiesS[2];  // CONNECTION, WEATHER_STATUS
        ISwitchVectorProperty WeatherPropertiesSP;
        
        // Enable/disable notifications
        ISwitch NotificationS[2]; // ON/OFF
        ISwitchVectorProperty NotificationSP;
        
        // Test notification
        ISwitch TestNotificationS[1];
        ISwitchVectorProperty TestNotificationSP;
        
        // Message batching configuration
        INumber BatchConfigN[2];
        INumberVectorProperty BatchConfigNP;
        
        // Internal state
        std::mutex mutex;
        std::chrono::system_clock::time_point lastBatchTime;

        // Structure pour stocker un message avec son timestamp
        struct QueuedMessage {
            std::string message;
            std::chrono::system_clock::time_point timestamp;
            
            QueuedMessage(const std::string& msg) : 
                message(msg), 
                timestamp(std::chrono::system_clock::now()) 
            {}
        };

        // File d'attente des messages
        std::queue<QueuedMessage> messageQueue;
        
        // Paramètres de rate limiting
        static constexpr int MAX_MESSAGES_PER_MINUTE = 30;
        static constexpr std::chrono::milliseconds MIN_DELAY_BETWEEN_MESSAGES{200}; // 5 par seconde
        
        // Timestamp du dernier message envoyé
        std::chrono::system_clock::time_point lastMessageTime;
        
        // Compteur de messages sur la dernière minute
        int messageCountLastMinute{0};
        
        // Timer pour le traitement de la file d'attente
        void TimerHit() override;
        
        // Gestion de la file d'attente
        void processMessageQueue();
        bool queueDiscordMessage(const std::string &message);
        bool shouldSendBatch();

        // Device notification enable/disable switches
        ISwitch MountNotifyS[2];
        ISwitchVectorProperty MountNotifySP;

        ISwitch CCDNotifyS[2];
        ISwitchVectorProperty CCDNotifySP;

        ISwitch FocuserNotifyS[2];
        ISwitchVectorProperty FocuserNotifySP;

        ISwitch FilterWheelNotifyS[2];
        ISwitchVectorProperty FilterWheelNotifySP;

        ISwitch DomeNotifyS[2];
        ISwitchVectorProperty DomeNotifySP;

        ISwitch WeatherNotifyS[2];
        ISwitchVectorProperty WeatherNotifySP;

        // Helper function to check if notifications are enabled for a device type
        bool areNotificationsEnabled(const char* deviceType);
}; 