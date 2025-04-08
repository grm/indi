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

#pragma once

#include "defaultdevice.h"
#include "indipropertyswitch.h"
#include "indipropertytext.h"
#include "indipropertynumber.h"
#include "indipropertylight.h"
#include "connectionplugins/connectiontcp.h"
#include "indiweatherinterface.h"

namespace Connection
{
class TCP;
}

class UPSSafety : public INDI::DefaultDevice, public INDI::WeatherInterface
{
    public:
        UPSSafety();
        virtual ~UPSSafety();

        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool saveConfigItems(FILE *fp) override;

    protected:
        virtual const char *getDefaultName() override;
        virtual bool Connect() override;
        virtual bool Disconnect() override;
        virtual void TimerHit() override;
        
        // Implement WeatherInterface method
        virtual IPState updateWeather() override;

        // Polling period
        virtual uint32_t getCurrentPollingPeriod() { return 2000; }

    private:
        // TCP Connection
        Connection::TCP *tcpConnection { nullptr };
        int PortFD { -1 };
        bool Handshake();
        bool sendCommand(const std::string &command, std::vector<std::string> &response);

        // UPS Connection Settings
        INDI::PropertyText UPSConnectionTP {1};
        enum
        {
            UPS_NAME,
        };

        // UPS Status
        INDI::PropertyLight StatusLP {4};
        enum
        {
            STATUS_ONLINE,
            STATUS_CHARGING,
            STATUS_DISCHARGING,
            STATUS_LOW_BATTERY,
        };

        // Battery Info
        INDI::PropertyNumber BatteryNP {3};
        enum
        {
            BATTERY_CHARGE,
            BATTERY_VOLTAGE,
            N_BATTERY
        };

        // UPS Info
        INDI::PropertyText InfoTP {1};
        enum
        {
            INFO_STATUS,
            N_INFO
        };

        // Safety Triggers
        INDI::PropertyNumber SafetyTriggersNP {2};
        enum
        {
            TRIGGER_LOW_BATTERY,
            TRIGGER_CRITICAL_BATTERY,
        };

        // Safety Actions
        INDI::PropertyText SafetyActionsTP {2};
        enum
        {
            ACTION_LOW_BATTERY,
            ACTION_CRITICAL_BATTERY,
        };

        // Safety Parameter that other devices can monitor
        INDI::PropertyNumber SafetyParameterNP {1};
        enum
        {
            PARAM_SAFETY_STATUS,
        };

        // Helper functions
        void updateUPSStatus();
        void updateUPSInfo();
        void processUPSState(const char *status);
        void checkSafetyTriggers();

        // Safety trigger flags
        bool m_LowBatteryTriggered {false};
        bool m_CriticalBatteryTriggered {false};

        // Constants
        static constexpr const size_t BUFFER_SIZE = 1024;

        template<typename T>
        bool extractNUTValue(const std::vector<std::string> &response, const std::string &varName, T &value, std::string &strValue);
}; 