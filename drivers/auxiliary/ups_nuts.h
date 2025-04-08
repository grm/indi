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
#include "indiproperty.h"
#include "inditimer.h"
#include "indiweatherinterface.h"
#include "connectionplugins/connectiontcp.h"

class UPSSafety : public INDI::DefaultDevice, public INDI::WeatherInterface
{
    public:
        UPSSafety();
        virtual ~UPSSafety() = default;

        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool saveConfigItems(FILE *fp) override;

    protected:
        const char *getDefaultName() override;
        bool Connect() override;
        bool Disconnect() override;
        void TimerHit() override;
        IPState updateWeather() override;

        bool isSimulation() { return SimulationSP[1].getState() == ISS_ON; }

    private:
        // TCP Connection
        Connection::TCP *tcpConnection{nullptr};

        // Simulation
        INDI::PropertySwitch SimulationSP{2};

        // Simulation Settings
        INDI::PropertyNumber SimulationNP{2};

        // UPS Connection Settings
        INDI::PropertyText UPSConnectionTP{1};

        // UPS Status
        INDI::PropertyLight StatusLP{4};

        // UPS Info
        INDI::PropertyText InfoTP{1};

        // Connection
        int PortFD{-1};
        static constexpr size_t BUFFER_SIZE{256};
        bool sendCommand(const std::string &command, std::vector<std::string> &response);
        bool Handshake();
        void processUPSState(const char *status);
        template<typename T>
        bool extractNUTValue(const std::vector<std::string> &response, const std::string &varName, T &value, std::string &strValue);
}; 