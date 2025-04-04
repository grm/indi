/*
  Focus Lynx INDI driver
  Copyright (C) 2015 Jasem Mutlaq (mutlaqja@ikarustech.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "focuslynxbase.h"

/************************************************************************************
 *
* ***********************************************************************************/
FocusLynxBase::FocusLynxBase(const char *target)
{
    INDI_UNUSED(target);
}

/************************************************************************************
 *
* ***********************************************************************************/
FocusLynxBase::FocusLynxBase()
{
    setVersion(VERSION, SUBVERSION);

    lynxModels["Optec TCF-Lynx 2"] = "OA";
    lynxModels["Optec TCF-Lynx 3"] = "OB";

    // "OC" is now reserved, it is hard coded into focusers that use it
    // Although it can be selected it should not be
    // lynxModels["Optec TCF-Lynx 2 with Extended Travel"] = "OC";
    lynxModels["Optec Fast Focus Secondary Focuser"] = "OD";

    // "OE" and "OF" are reserved, no models that are not "OA" or "OB" have been deployed
    // lynxModels["Optec TCF-S Classic converted"] = "OE";
    // lynxModels["Optec TCF-S3 Classic converted"] = "OF";

    // "OG" Gemini is reserved. It is used to identify the Optec Gemini
    // lynxModels["Optec Gemini (reserved for future use)"] = "OG";

    lynxModels["Optec Leo"] = "OI";
    lynxModels["Optec Leo High-Torque"] = "OJ";
    lynxModels["Optec Sagitta"] = "OK";
    lynxModels["Optec Sagitta 2"] = "OL";

    // These are generic for all Optec QuickSync and FeatherTouch HSM models
    lynxModels["QuickSync / HSM Hi-Torque"] = "FA";
    lynxModels["QuickSync / HSM Hi-Speed"] = "FB";

    // "FC" is reserved, it has not been deployed and is covered by "FA" and "FB"
    // lynxModels["FocusLynx QuickSync SV (reserved for future use)"] = "FC";

    // These are generic for all Optec DirectSync and FeatherTouch PDMS models
    lynxModels["DirectSync / PDMS Hi-Torque"] = "FD";
    lynxModels["DirectSync / PDMS Hi-Speed"] = "FF";

    // JM 2019-09-27: This was added after the discussion here
    // https://www.indilib.org/forum/focusers-filter-wheels/5739-starlight-instruments-focuser-boss-ii-hsm20.html
    // DVN 2021-11-15: Covered by the generic Device Type FA and FB.
    // Continued: duplicating device types can result in the wrong type being displayed in the selection box
    // lynxModels["FeatureTouch HSM Hi-Torque"] = "FA";
    // lynxModels["FeatureTouch HSM Hi-Speed"] = "FB";

    // FE is deprecated, future firmwares will automatically switch to generic device type FD
    lynxModels["FeatherTouch Motor PDMS"] = "FE";

    lynxModels["FeatherTouch Microtouch MSM Hi-Speed"] = "SO";
    lynxModels["FeatherTouch Microtouch MSM Hi-Torque"] = "SP";
    lynxModels["Starlight Instruments - FTM with MicroTouch"] = "SQ";

    //TA is deprecated, future firmwares will automatically switch to generic device type FA
    lynxModels["Televue Focuser"] = "TA";

    lynxModels["Unipolar motor (Robo-Focus)"] = "RA";

    ModelS = nullptr;

    focusMoveRequest = 0;
    simPosition      = 0;

    // Can move in Absolute & Relative motions, can AbortFocuser motion, can sync, and has variable speed.
    FI::SetCapability(FOCUSER_CAN_ABORT    |
                      FOCUSER_CAN_ABS_MOVE |
                      FOCUSER_CAN_REL_MOVE |
                      FOCUSER_CAN_SYNC     |
                      FOCUSER_CAN_REVERSE  |
                      FOCUSER_HAS_BACKLASH);

    canHome = false;
    isHoming   = false;

    simStatus[STATUS_MOVING]   = ISS_OFF;
    simStatus[STATUS_HOMING]   = ISS_OFF;
    simStatus[STATUS_HOMED]    = ISS_OFF;
    simStatus[STATUS_FFDETECT] = ISS_OFF;
    simStatus[STATUS_TMPPROBE] = ISS_ON;
    simStatus[STATUS_REMOTEIO] = ISS_ON;
    simStatus[STATUS_HNDCTRL]  = ISS_ON;
    simStatus[STATUS_REVERSE]  = ISS_OFF;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::initProperties()
{
    INDI::Focuser::initProperties();

    // Focuser temperature
    IUFillNumber(&TemperatureN[0], "TEMPERATURE", "Celsius", "%6.2f", -50, 70., 0., 0.);
    IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature",
                       MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Enable/Disable temperature compensation
    IUFillSwitch(&TemperatureCompensateS[INDI_ENABLED], "INDI_ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateS[INDI_DISABLED], "INDI_DISABLED", "Disabled", ISS_ON);
    IUFillSwitchVector(&TemperatureCompensateSP, TemperatureCompensateS, 2, getDeviceName(), "T. COMPENSATION",
                       "T. Compensation",
                       FOCUS_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Enable/Disable temperature compensation on start
    IUFillSwitch(&TemperatureCompensateOnStartS[0], "Enable", "Enable", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateOnStartS[1], "Disable", "Disable", ISS_ON);
    IUFillSwitchVector(&TemperatureCompensateOnStartSP, TemperatureCompensateOnStartS, 2, getDeviceName(),
                       "T. COMPENSATION @START", "T. Compensation @Start", FOCUS_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Enable/Disable temperature Mode
    IUFillSwitch(&TemperatureCompensateModeS[0], "A", "A", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateModeS[1], "B", "B", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateModeS[2], "C", "C", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateModeS[3], "D", "D", ISS_OFF);
    IUFillSwitch(&TemperatureCompensateModeS[4], "E", "E", ISS_OFF);
    IUFillSwitchVector(&TemperatureCompensateModeSP, TemperatureCompensateModeS, 5, getDeviceName(), "COMPENSATE MODE",
                       "Compensate Mode", FOCUS_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillNumber(&TemperatureParamN[0], "T. Coefficient", "", "%.f", -9999, 9999, 100., 0.);
    IUFillNumber(&TemperatureParamN[1], "T. Intercept", "", "%.f", -32766, 32766, 100., 0.);
    IUFillNumberVector(&TemperatureParamNP, TemperatureParamN, 2, getDeviceName(), "T. PARAMETERS", "Mode Parameters",
                       FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    // Focuser Step Size
    // An early command doc listed this as 10000 instead of the correct 1000. Because this is informational only (not used within the controller)
    // fixing it should have no consequences.
    IUFillNumber(&StepSizeN[0], "1000*microns/step", "", "%.f", 0, 65535, 0., 0);
    IUFillNumberVector(&StepSizeNP, StepSizeN, 1, getDeviceName(), "STEP SIZE", "Step Size", FOCUS_SETTINGS_TAB, IP_RW, 0,
                       IPS_IDLE);

    // Reset to Factory setting
    IUFillSwitch(&ResetS[0], "Factory", "Factory", ISS_OFF);
    IUFillSwitchVector(&ResetSP, ResetS, 1, getDeviceName(), "RESET", "Reset", FOCUS_SETTINGS_TAB, IP_RW, ISR_ATMOST1, 0,
                       IPS_IDLE);

    // Go to home/center
    IUFillSwitch(&GotoS[GOTO_CENTER], "Center", "Center", ISS_OFF);
    IUFillSwitch(&GotoS[GOTO_HOME], "Home", "Home", ISS_OFF);
    IUFillSwitchVector(&GotoSP, GotoS, 2, getDeviceName(), "GOTO", "Goto", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    // List all supported models
    std::map<std::string, std::string>::iterator iter;
    int nModels = 1;
    ModelS = static_cast<ISwitch *>(malloc(sizeof(ISwitch)));
    // Need to be able to select no focuser to avoid troubles with Ekos
    IUFillSwitch(ModelS, "No Focuser", "No Focuser", ISS_ON);
    for (iter = lynxModels.begin(); iter != lynxModels.end(); ++iter)
    {
        ISwitch * buffer = static_cast<ISwitch *>(realloc(ModelS, (nModels + 1) * sizeof(ISwitch)));
        if (!buffer)
        {
            free(ModelS);
            return false;
        }
        else
            ModelS = buffer;
        IUFillSwitch(ModelS + nModels, (iter->first).c_str(), (iter->first).c_str(), ISS_OFF);

        nModels++;
    }
    IUFillSwitchVector(&ModelSP, ModelS, nModels, getDeviceName(), "MODEL", "Model", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    // Status indicators
    IUFillLight(&StatusL[STATUS_MOVING], "Is Moving", "", IPS_IDLE);
    IUFillLight(&StatusL[STATUS_HOMING], "Is Homing", "", IPS_IDLE);
    IUFillLight(&StatusL[STATUS_HOMED], "Is Homed", "", IPS_IDLE);
    IUFillLight(&StatusL[STATUS_FFDETECT], "FF Detect", "", IPS_IDLE);
    IUFillLight(&StatusL[STATUS_TMPPROBE], "Tmp Probe", "", IPS_IDLE);
    IUFillLight(&StatusL[STATUS_REMOTEIO], "Remote IO", "", IPS_IDLE);
    IUFillLight(&StatusL[STATUS_HNDCTRL], "Hnd Ctrl", "", IPS_IDLE);
    IUFillLight(&StatusL[STATUS_REVERSE], "Reverse", "", IPS_IDLE);
    IUFillLightVector(&StatusLP, StatusL, 8, getDeviceName(), "STATUS", "Status", FOCUS_STATUS_TAB, IPS_IDLE);

    // Focus name configure in the HUB
    IUFillText(&HFocusNameT[0], "FocusName", "Focuser name", "");
    IUFillTextVector(&HFocusNameTP, HFocusNameT, 1, getDeviceName(), "FOCUSNAME", "Focuser", FOCUS_SETTINGS_TAB, IP_RW, 0,
                     IPS_IDLE);

    // Led intensity value
    IUFillNumber(&LedN[0], "Intensity", "", "%.f", 0, 100, 5., 0.);
    IUFillNumberVector(&LedNP, LedN, 1, getDeviceName(), "LED", "Led", FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);
    //simPosition = FocusAbsPosNP[0].getValue();

    addAuxControls();

    return true;
}

/************************************************************************************
 *
* ***********************************************************************************/
void FocusLynxBase::ISGetProperties(const char *dev)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    INDI::Focuser::ISGetProperties(dev);

    defineProperty(&ModelSP);
    if (isSimulation())
        loadConfig(true, "Model");
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::updateProperties()
{
    // For homing focusers the vector is set to RO, as we get value from the HUB

    if(canHome)
    {
        FocusMaxPosNP.setPermission(IP_RO);
    }
    else
    {
        FocusMaxPosNP.setPermission(IP_RW);
    }

    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(&HFocusNameTP);

        defineProperty(&TemperatureNP);
        defineProperty(&TemperatureCompensateModeSP);
        defineProperty(&TemperatureParamNP);
        defineProperty(&TemperatureCompensateSP);
        defineProperty(&TemperatureCompensateOnStartSP);

        //        defineProperty(&FocusBacklashSP);
        //        defineProperty(&FocusBacklashNP);

        //defineProperty(&MaxTravelNP);

        defineProperty(&StepSizeNP);

        defineProperty(&ResetSP);
        //defineProperty(&ReverseSP);
        defineProperty(&StatusLP);

        if (getFocusConfig() && getFocusTemp())
            LOG_INFO("FocusLynx parameters updated, focuser ready for use.");
        else
        {
            LOG_ERROR("Failed to retrieve focuser configuration settings...");
            return false;
        }
    }
    else
    {
        deleteProperty(TemperatureNP.name);
        deleteProperty(TemperatureCompensateModeSP.name);
        deleteProperty(TemperatureCompensateSP.name);
        deleteProperty(TemperatureParamNP.name);
        deleteProperty(TemperatureCompensateOnStartSP.name);

        //        deleteProperty(FocusBacklashSP.name);
        //        deleteProperty(FocusBacklashNP.name);

        //deleteProperty(MaxTravelNP.name);
        deleteProperty(StepSizeNP.name);

        deleteProperty(ResetSP.name);
        deleteProperty(GotoSP.name);
        //deleteProperty(ReverseSP.name);

        deleteProperty(StatusLP.name);
        deleteProperty(HFocusNameTP.name);
    }

    return true;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::Handshake()
{
    if (ack())
    {
        LOG_INFO("FocusLynx is online. Getting focus parameters...");
        return true;
    }

    LOG_INFO("Error retrieving data from FocusLynx, please ensure FocusLynxBase controller is "
             "powered and the port is correct.");
    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
const char *FocusLynxBase::getDefaultName()
{
    // Has to be overridden by child instance
    return "FocusLynxBase";
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Models
        if (strcmp(ModelSP.name, name) == 0)
        {
            IUUpdateSwitch(&ModelSP, states, names, n);
            ModelSP.s = IPS_OK;
            IDSetSwitch(&ModelSP, nullptr);
            if (isConnected())
            {
                setDeviceType(IUFindOnSwitchIndex(&ModelSP));
                LOG_INFO("Focuser model set. Please disconnect and reconnect now...");
            }
            else
                LOG_INFO("Focuser model set. Please connect now...");

            //Read the config for this new model form the HUB
            getFocusConfig();

            return true;
        }

        // Temperature Compensation
        if (strcmp(TemperatureCompensateSP.name, name) == 0)
        {
            int prevIndex = IUFindOnSwitchIndex(&TemperatureCompensateSP);
            IUUpdateSwitch(&TemperatureCompensateSP, states, names, n);
            if (setTemperatureCompensation(TemperatureCompensateS[0].s == ISS_ON))
            {
                TemperatureCompensateSP.s = IPS_OK;
            }
            else
            {
                IUResetSwitch(&TemperatureCompensateSP);
                TemperatureCompensateSP.s           = IPS_ALERT;
                TemperatureCompensateS[prevIndex].s = ISS_ON;
            }

            IDSetSwitch(&TemperatureCompensateSP, nullptr);
            return true;
        }

        // Temperature Compensation on Start
        if (!strcmp(TemperatureCompensateOnStartSP.name, name))
        {
            int prevIndex = IUFindOnSwitchIndex(&TemperatureCompensateOnStartSP);
            IUUpdateSwitch(&TemperatureCompensateOnStartSP, states, names, n);
            if (setTemperatureCompensationOnStart(TemperatureCompensateOnStartS[0].s == ISS_ON))
            {
                TemperatureCompensateOnStartSP.s = IPS_OK;
            }
            else
            {
                IUResetSwitch(&TemperatureCompensateOnStartSP);
                TemperatureCompensateOnStartSP.s           = IPS_ALERT;
                TemperatureCompensateOnStartS[prevIndex].s = ISS_ON;
            }

            IDSetSwitch(&TemperatureCompensateOnStartSP, nullptr);
            return true;
        }

        // Temperature Compensation Mode
        if (!strcmp(TemperatureCompensateModeSP.name, name))
        {
            int prevIndex = IUFindOnSwitchIndex(&TemperatureCompensateModeSP);
            IUUpdateSwitch(&TemperatureCompensateModeSP, states, names, n);
            char mode = IUFindOnSwitchIndex(&TemperatureCompensateModeSP) + 'A';
            if (setTemperatureCompensationMode(mode))
            {
                TemperatureCompensateModeSP.s = IPS_OK;
            }
            else
            {
                IUResetSwitch(&TemperatureCompensateModeSP);
                TemperatureCompensateModeSP.s           = IPS_ALERT;
                TemperatureCompensateModeS[prevIndex].s = ISS_ON;
            }

            IDSetSwitch(&TemperatureCompensateModeSP, nullptr);
            return true;
        }

        // Backlash enable/disable
        //        if (!strcmp(FocusBacklashSP.name, name))
        //        {
        //            int prevIndex = IUFindOnSwitchIndex(&FocusBacklashSP);
        //            IUUpdateSwitch(&FocusBacklashSP, states, names, n);
        //            if (setBacklashCompensation(FocusBacklashS[INDI_ENABLED].s == ISS_ON))
        //            {
        //                FocusBacklashSP.s = IPS_OK;
        //            }
        //            else
        //            {
        //                IUResetSwitch(&FocusBacklashSP);
        //                FocusBacklashSP.s           = IPS_ALERT;
        //                FocusBacklashS[prevIndex].s = ISS_ON;
        //            }

        //            FocusBacklashSP.apply();
        //            return true;
        //        }

        // Reset to Factory setting
        if (strcmp(ResetSP.name, name) == 0)
        {
            IUResetSwitch(&ResetSP);
            if (resetFactory())
                ResetSP.s = IPS_OK;
            else
                ResetSP.s = IPS_ALERT;

            IDSetSwitch(&ResetSP, nullptr);
            return true;
        }

        // Go to home/center
        if (!strcmp(GotoSP.name, name))
        {
            IUUpdateSwitch(&GotoSP, states, names, n);

            if (GotoS[GOTO_HOME].s == ISS_ON)
            {
                if (home())
                    GotoSP.s = IPS_BUSY;
                else
                    GotoSP.s = IPS_ALERT;
            }
            else
            {
                if (center())
                    GotoSP.s = IPS_BUSY;
                else
                    GotoSP.s = IPS_ALERT;
            }

            IDSetSwitch(&GotoSP, nullptr);
            return true;
        }

        // Reverse Direction
        //        if (!strcmp(ReverseSP.name, name))
        //        {
        //            IUUpdateSwitch(&ReverseSP, states, names, n);

        //            if (reverse(ReverseS[0].s == ISS_ON))
        //                ReverseSP.s = IPS_OK;
        //            else
        //                ReverseSP.s = IPS_ALERT;

        //            IDSetSwitch(&ReverseSP, nullptr);
        //            return true;
        //        }

    }
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Set device nickname to the HUB itself
        if (!strcmp(name, HFocusNameTP.name))
        {
            IUUpdateText(&HFocusNameTP, texts, names, n);
            if (setDeviceNickname(HFocusNameT[0].text))
                HFocusNameTP.s = IPS_OK;
            else
                HFocusNameTP.s = IPS_ALERT;
            IDSetText(&HFocusNameTP, nullptr);
            return true;
        }
    }
    return INDI::Focuser::ISNewText(dev, name, texts, names, n);
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Temperature Coefficient & Inceptions
        if (!strcmp(TemperatureParamNP.name, name))
        {
            IUUpdateNumber(&TemperatureParamNP, values, names, n);

            char mode = static_cast<char>(65 + IUFindOnSwitchIndex(&TemperatureCompensateModeSP));
            if (!setTemperatureCompensationCoeff(mode, TemperatureParamN[0].value) ||
                    !setTemperatureInceptions(mode, TemperatureParamN[1].value))
            {
                LOG_ERROR("Failed to write temperature coefficient or intercept");
                TemperatureParamNP.s = IPS_ALERT;
                IDSetNumber(&TemperatureParamNP, nullptr);
                return false;
            }

            TemperatureParamNP.s = IPS_OK;
            getFocusTemp();

            return true;
        }

        // Backlash Value
        //        if (!strcmp(FocusBacklashNP.name, name))
        //        {
        //            IUUpdateNumber(&FocusBacklashNP, values, names, n);
        //            if (setFocusBacklashSteps(FocusBacklashNP.apply();) == false)
        //            {
        //                LOG_ERROR("Failed to set temperature coefficients.");
        //                FocusBacklashNP.s = IPS_ALERT;
        //                IDSetNumber(&FocusBacklashNP, nullptr);
        //                return false;
        //            }

        //            FocusBacklashNP.s = IPS_OK;
        //            IDSetNumber(&FocusBacklashNP, nullptr);
        //            return true;
        //        }

        // Sync
        //        if (strcmp(SyncNP.name, name) == 0)
        //        {
        //            IUUpdateNumber(&SyncNP, values, names, n);
        //            if (sync(SyncN[0].value) == false)
        //            {
        //                LOG_ERROR("Failed to set the actual value.");
        //                SyncNP.s = IPS_ALERT;
        //                IDSetNumber(&SyncNP, nullptr);
        //                return false;
        //            }
        //            else
        //                SyncNP.s = IPS_OK;

        //            IDSetNumber(&SyncNP, nullptr);
        //            return true;
        //        }

        // StepSize
        if (strcmp(StepSizeNP.name, name) == 0)
        {
            IUUpdateNumber(&StepSizeNP, values, names, n);
            if (setStepSize(StepSizeN[0].value) == false)
            {
                LOG_ERROR("Failed to set the actual value.");
                StepSizeNP.s = IPS_ALERT;
                IDSetNumber(&StepSizeNP, nullptr);
                return false;
            }
            else
                StepSizeNP.s = IPS_OK;

            IDSetNumber(&StepSizeNP, nullptr);
            return true;
        }

        // Max Travel if relative focusers
        //        if (strcmp(MaxTravelNP.name, name) == 0)
        //        {
        //            IUUpdateNumber(&MaxTravelNP, values, names, n);

        //            if (setMaxTravel(MaxTravelN[0].value) == false)
        //                MaxTravelNP.s = IPS_ALERT;
        //            else
        //            {
        //                MaxTravelNP.s = IPS_OK;
        //                FocusAbsPosNP[0].setMax(SyncNP[0].setMax(MaxTravelN[0].value));
        //                FocusAbsPosNP[0].setStep(SyncNP[0].setStep((MaxTravelN[0].value / 50.0)));

        //                FocusAbsPosNP.updateMinMax();
        //                IUUpdateMinMax(&SyncNP);

        //                IDSetNumber(&MaxTravelNP, nullptr);

        //                LOGF_INFO("Focuser absolute limits: min (%g) max (%g)", FocusAbsPosNP[0].getMin(),
        //                       FocusAbsPosNP[0].getMax());
        //            }
        //            return true;
        //        }

        // Set LED intensity to the HUB itself via function setLedLevel()
        if (!strcmp(LedNP.name, name))
        {
            IUUpdateNumber(&LedNP, values, names, n);
            if (setLedLevel(LedN[0].value))
                LedNP.s = IPS_OK;
            else
                LedNP.s = IPS_ALERT;
            LOGF_INFO("Focuser LED level intensity : %f", LedN[0].value);
            IDSetNumber(&LedNP, nullptr);
            return true;
        }
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::ack()
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sHELLO>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        const char *focusName = IUFindOnSwitch(&ModelSP)->label;
        strncpy(response, focusName, LYNX_MAX);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        LOGF_INFO("%s is detected.", response);

        return true;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::getFocusConfig()
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;
    char key[16];

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sGETCONFIG>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        if (!strcmp(getFocusTarget(), "F1"))
            strncpy(response, "CONFIG1", 16);
        else
            strncpy(response, "CONFIG2", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        if ((strcmp(response, "CONFIG1")) && (strcmp(response, "CONFIG2")))
            return false;
    }

    memset(response, 0, sizeof(response));

    // Nickname
    if (isSimulation())
    {
        snprintf(response, sizeof(response), "NickName=Focuser#%s\n", getFocusTarget());
        nbytes_read = strlen(response);
    }
    else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }
    response[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES (%s)", response);

    char nickname[16];
    int rc = sscanf(response, "%15[^=]=%15[^\n]s", key, nickname);

    if (rc != 2)
        return false;

    IUSaveText(&HFocusNameT[0], nickname);
    IDSetText(&HFocusNameTP, nullptr);

    HFocusNameTP.s = IPS_OK;
    IDSetText(&HFocusNameTP, nullptr);

    memset(response, 0, sizeof(response));

    // Get Max Position
    if (isSimulation())
    {
        // Default value from non-homing focusers
        snprintf(response, 32, "Max Pos = %06d\n", 65535);
        nbytes_read = strlen(response);
    }
    else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }
    response[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES (%s)", response);

    uint32_t maxPos = 0;
    rc = sscanf(response, "%15[^=]=%d", key, &maxPos);
    if (rc == 2)
    {
        FocusAbsPosNP[0].setMin(0);
        FocusAbsPosNP[0].setMax(maxPos);
        FocusAbsPosNP[0].setStep(maxPos / 50.0);

        FocusSyncNP[0].setMin(0);
        FocusSyncNP[0].setMax(maxPos);
        FocusSyncNP[0].setStep(maxPos / 50.0);

        FocusRelPosNP[0].setMin(0);
        FocusRelPosNP[0].setMax(maxPos / 2);
        FocusRelPosNP[0].setStep(maxPos / 100.0);

        FocusAbsPosNP.updateMinMax();
        FocusRelPosNP.updateMinMax();
        FocusSyncNP.updateMinMax();

        FocusMaxPosNP.setState(IPS_OK);
        FocusMaxPosNP[0].setValue(maxPos);
        FocusMaxPosNP.apply();

    }
    else
        return false;

    memset(response, 0, sizeof(response));

    // Get Device Type
    if (isSimulation())
    {
        // In simulation each focuser is different, one Absolute and one relative
        if (strcmp(getFocusTarget(), "F2"))
            snprintf(response, 32, "Dev Type = %s\n", "OA");
        else
            snprintf(response, 32, "Dev Type = %s\n", "SO");
        nbytes_read = strlen(response);
    }
    else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }
    response[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES (%s)", response);

    // Don't process the response if isSimulation active, Value read from saved config
    if (!isSimulation())
    {
        //Extract the code from the response value
        std::string tmpString;
        tmpString.assign(response + 11, 2);
        int count = 0;

        //As "ZZ" is not exist in lynxModel, not need interator, 'No focuser' is known as first in ModelS
        if(tmpString != "ZZ")
        {
            //Reset the Home / Center function
            deleteProperty(GotoSP.name);
            //All models that can home have the first character of the device type as the letter 'O'
            canHome = tmpString[0] == 'O';

            if(canHome)
            {
                //Homing focusers can not sync
                deleteProperty(FocusSyncNP);
                GotoSP.nsp = 2;
            }
            else
            {
                //Non-Homing focusers can sync
                defineProperty(FocusSyncNP);
                GotoSP.nsp = 1;
            }

            //Reactivate the Home / Center Property
            defineProperty(&GotoSP);

            // If not 'No Focuser' then do iterator
            // iterate through all elements in std::map<std::string, std::string> and search the index from the code.
            std::map<std::string, std::string>::iterator it = lynxModels.begin();
            while(it != lynxModels.end())
            {
                count++;
                if (it->second == tmpString)
                    break;
                it++;
            }
        }

        // as different focuser could have the same code in the HUB, we are not able to find the correct name in the list of focuser.
        // The first one would be show as the item.
        IUResetSwitch(&ModelSP);
        ModelS[count].s = ISS_ON;
        IDSetSwitch(&ModelSP, nullptr);

        LOGF_DEBUG("Index focuser : %d", count);
    } // end if (!isSimulation)

    // Get Status Parameters

    memset(response, 0, sizeof(response));

    // Temperature information processed on function getFocusTemp(), do nothing with related response

    // Temperature Compensation On
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Temperature Coeff A
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Temperature Coeff B
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Temperature Coeff C
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Temperature Coeff D
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Temperature Coeff E
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Temperature Compensation Mode
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Backlash Compensation
    if (isSimulation())
    {
        snprintf(response, 32, "BLC En = %d\n", FocusBacklashSP[INDI_ENABLED].getState() == ISS_ON ? 1 : 0);
        nbytes_read = strlen(response);
    }
    else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }
    response[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES (%s)", response);

    int BLCCompensate;
    rc = sscanf(response, "%15[^=]=%d", key, &BLCCompensate);
    if (rc != 2)
        return false;

    FocusBacklashSP.reset();
    FocusBacklashSP[INDI_ENABLED].setState(BLCCompensate ? ISS_ON : ISS_OFF);
    FocusBacklashSP[INDI_DISABLED].setState(BLCCompensate ? ISS_OFF : ISS_ON);
    FocusBacklashSP.setState(IPS_OK);
    FocusBacklashSP.apply();

    // Backlash Value
    memset(response, 0, sizeof(response));
    if (isSimulation())
    {
        snprintf(response, 32, "BLC Stps = %d\n", 50);
        nbytes_read = strlen(response);
    }
    else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }
    response[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES (%s)", response);

    int BLCValue;
    rc = sscanf(response, "%15[^=]=%d", key, &BLCValue);
    if (rc != 2)
        return false;

    FocusBacklashNP[0].setValue(BLCValue);
    FocusBacklashNP.setState(IPS_OK);
    FocusBacklashNP.apply();

    // Led brightness
    memset(response, 0, sizeof(response));
    if (isSimulation())
    {
        snprintf(response, 32, "LED Brt = %d\n", 75);
        nbytes_read = strlen(response);
    }
    else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }
    response[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES (%s)", response);

    int LEDBrightness;
    rc = sscanf(response, "%15[^=]=%d", key, &LEDBrightness);
    if (rc != 2)
        return false;

    LedN[0].value = LEDBrightness;
    LedNP.s       = IPS_OK;
    IDSetNumber(&LedNP, nullptr);

    // Temperature Compensation on Start
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }

    // Home on start?
    // JM 2021.05.09: This appears to be a new addition in firmware v1.2.0 (2019).
    // We should ignore if END is received instead.
    if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        tty_error_msg(errcode, errmsg, MAXRBUF);
        LOGF_ERROR("%s", errmsg);
        return false;
    }
    // If END, then ignore
    else if (strncmp(response, "END", 3))
    {
        int homeOnStart;
        rc = sscanf(response, "%15[^=]=%d", key, &homeOnStart);
        if (rc != 2)
            return false;

        m_HomeOnStart = homeOnStart == 1;
    }

    // If last response was END, then ignore
    if (strncmp(response, "END", 3))
    {
        // END is reached
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            strncpy(response, "END\n", 16);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';

        // Display the response to be sure to have read the complete TTY Buffer.
        LOGF_DEBUG("RES (%s)", response);

        if (strcmp(response, "END"))
            return false;
    }

    tcflush(PortFD, TCIFLUSH);

    configurationComplete = true;

    return true;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::getFocusStatus()
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;
    char key[16];

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sGETSTATUS>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        if (!strcmp(getFocusTarget(), "F1"))
            strncpy(response, "STATUS1", 16);
        else
            strncpy(response, "STATUS2", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        //tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        if (!((!strcmp(response, "STATUS1")) && (!strcmp(getFocusTarget(), "F1"))) && !((!strcmp(response, "STATUS2"))
                && (!strcmp(getFocusTarget(), "F2"))))
        {
            tcflush(PortFD, TCIFLUSH);
            return false;
        }

        // Get Temperature
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            //strncpy(response, "Temp(C) = +21.7\n", 16); // #PS: for string literal, use strcpy
            strcpy(response, "Temp(C) = +21.7\n");
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (nbytes_read > 0)
            response[nbytes_read - 1] = '\0'; // remove last character (new line)

        LOGF_DEBUG("RES (%s)", response);

        float temperature = 0;
        int rc            = sscanf(response, "%15[^=]=%f", key, &temperature);
        if (rc == 2)
        {
            TemperatureN[0].value = temperature;
            IDSetNumber(&TemperatureNP, nullptr);
        }
        else
        {
            if (TemperatureNP.s != IPS_ALERT)
            {
                TemperatureNP.s = IPS_ALERT;
                IDSetNumber(&TemperatureNP, nullptr);
            }
        }

        // Get Current Position
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "Curr Pos = %06d\n", simPosition);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        uint32_t currPos = 0;
        rc               = sscanf(response, "%15[^=]=%d", key, &currPos);
        if (rc == 2)
        {
            FocusAbsPosNP[0].setValue(currPos);
            FocusAbsPosNP.apply();
        }
        else
            return false;

        // Get Target Position
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "Targ Pos = %06d\n", targetPosition);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        // Get Status Parameters

        // #1 is Moving?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "Is Moving = %d\n", (simStatus[STATUS_MOVING] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int isMoving;
        rc = sscanf(response, "%15[^=]=%d", key, &isMoving);
        if (rc != 2)
            return false;

        StatusL[STATUS_MOVING].s = isMoving ? IPS_BUSY : IPS_IDLE;

        // #2 is Homing?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "Is Homing = %d\n", (simStatus[STATUS_HOMING] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int _isHoming;
        rc = sscanf(response, "%15[^=]=%d", key, &_isHoming);
        if (rc != 2)
            return false;

        StatusL[STATUS_HOMING].s = _isHoming ? IPS_BUSY : IPS_IDLE;
        // For syncing only focusers home is not applicable.
        if (canHome == false)
            StatusL[STATUS_HOMING].s = IPS_IDLE;

        // We set that isHoming in process, but we don't set it to false here it must be reset in TimerHit
        if (StatusL[STATUS_HOMING].s == IPS_BUSY)
            isHoming = true;

        // #3 is Homed?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "Is Homed = %d\n", (simStatus[STATUS_HOMED] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int isHomed;
        rc = sscanf(response, "%15[^=]=%d", key, &isHomed);
        if (rc != 2)
            return false;

        StatusL[STATUS_HOMED].s = isHomed ? IPS_OK : IPS_IDLE;
        // For relative focusers home is not applicable.
        if (canHome == false)
            StatusL[STATUS_HOMED].s = IPS_IDLE;

        // #4 FF Detected?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "FFDetect = %d\n", (simStatus[STATUS_FFDETECT] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int FFDetect;
        rc = sscanf(response, "%15[^=]=%d", key, &FFDetect);
        if (rc != 2)
            return false;

        StatusL[STATUS_FFDETECT].s = FFDetect ? IPS_OK : IPS_IDLE;

        // #5 Temperature probe?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "TmpProbe = %d\n", (simStatus[STATUS_TMPPROBE] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int TmpProbe;
        rc = sscanf(response, "%15[^=]=%d", key, &TmpProbe);
        if (rc != 2)
            return false;

        StatusL[STATUS_TMPPROBE].s = TmpProbe ? IPS_OK : IPS_IDLE;

        // #6 Remote IO?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "RemoteIO = %d\n", (simStatus[STATUS_REMOTEIO] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int RemoteIO;
        rc = sscanf(response, "%15[^=]=%d", key, &RemoteIO);
        if (rc != 2)
            return false;

        StatusL[STATUS_REMOTEIO].s = RemoteIO ? IPS_OK : IPS_IDLE;

        // #7 Hand controller?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "Hnd Ctlr = %d\n", (simStatus[STATUS_HNDCTRL] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int HndCtlr;
        rc = sscanf(response, "%15[^=]=%d", key, &HndCtlr);
        if (rc != 2)
            return false;

        StatusL[STATUS_HNDCTRL].s = HndCtlr ? IPS_OK : IPS_IDLE;

        // #8 Reverse?
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "Reverse = %d\n", (simStatus[STATUS_REVERSE] == ISS_ON) ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int reverse;
        rc = sscanf(response, "%15[^=]=%d", key, &reverse);
        if (rc != 2)
            return false;

        StatusL[STATUS_REVERSE].s = reverse ? IPS_OK : IPS_IDLE;

        // If reverse is enable and switch shows disabled, let's change that
        // same thing is reverse is disabled but switch is enabled
        if ((reverse && FocusReverseSP[INDI_DISABLED].getState() == ISS_ON) || (!reverse && FocusReverseSP[INDI_ENABLED].getState() == ISS_ON))
        {
            FocusReverseSP.reset();
            FocusReverseSP[INDI_ENABLED].setState((reverse == 1) ? ISS_ON : ISS_OFF);
            FocusReverseSP[INDI_DISABLED].setState((reverse == 0) ? ISS_ON : ISS_OFF);
            FocusReverseSP.apply();
        }

        StatusLP.s = IPS_OK;
        IDSetLight(&StatusLP, nullptr);

        // END is reached
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            strncpy(response, "END\n", 16);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (nbytes_read > 0)
        {
            response[nbytes_read - 1] = '\0';

            // Display the response to be sure to have read the complete TTY Buffer.
            LOGF_DEBUG("RES (%s)", response);
            if (strcmp(response, "END"))
                return false;
        }

        tcflush(PortFD, TCIFLUSH);

        return true;
    }
    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::getFocusTemp()
{
    // Get value related to Temperature compensation

    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;
    char key[16];

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sGETTCI>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        if (!strcmp(getFocusTarget(), "F1"))
            strncpy(response, "TEMP COMP1", 16);
        else
            strncpy(response, "TEMP COMP2", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        if ((errcode = tty_write_string(PortFD, cmd,  &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        if ((strcmp(response, "TEMP COMP1")) && (strcmp(response, "TEMP COMP2")))
            return false;

        memset(response, 0, sizeof(response));

        // Temperature Compensation On?
        if (isSimulation())
        {
            snprintf(response, 32, "TComp ON = %d\n", TemperatureCompensateS[0].s == ISS_ON ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int TCompOn;
        int rc = sscanf(response, "%15[^=]=%d", key, &TCompOn);
        if (rc != 2)
            return false;

        IUResetSwitch(&TemperatureCompensateSP);
        TemperatureCompensateS[0].s = TCompOn ? ISS_ON : ISS_OFF;
        TemperatureCompensateS[1].s = TCompOn ? ISS_OFF : ISS_ON;
        TemperatureCompensateSP.s   = IPS_OK;
        IDSetSwitch(&TemperatureCompensateSP, nullptr);

        memset(response, 0, sizeof(response));

        // Temperature Compensation Mode
        if (isSimulation())
        {
            snprintf(response, 32, "TC Mode = %c\n", 'C');
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        char compensateMode;
        rc = sscanf(response, "%15[^=]= %c", key, &compensateMode);
        if (rc != 2)
        {
            if (rc == 1 && key[0] == 'T')
            {
                //If the controller does not support this it could be null. Assume A mode in this case.
                compensateMode = 'A';
            }
            else
            {
                return false;
            }
        }

        IUResetSwitch(&TemperatureCompensateModeSP);
        int index = compensateMode - 'A';
        if (index >= 0 && index <= 5)
        {
            TemperatureCompensateModeS[index].s = ISS_ON;
            TemperatureCompensateModeSP.s       = IPS_OK;
        }
        else
        {
            LOGF_ERROR("Invalid index %d for compensation mode.", index);
            TemperatureCompensateModeSP.s = IPS_ALERT;
        }

        IDSetSwitch(&TemperatureCompensateModeSP, nullptr);


        // Temperature Compensation on Start
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            snprintf(response, 32, "TC@Start = %d\n", TemperatureCompensateOnStartS[0].s == ISS_ON ? 1 : 0);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int TCOnStart;
        rc = sscanf(response, "%15[^=]=%d", key, &TCOnStart);
        if (rc != 2)
            return false;

        IUResetSwitch(&TemperatureCompensateOnStartSP);
        TemperatureCompensateOnStartS[0].s = TCOnStart ? ISS_ON : ISS_OFF;
        TemperatureCompensateOnStartS[1].s = TCOnStart ? ISS_OFF : ISS_ON;
        TemperatureCompensateOnStartSP.s   = IPS_OK;
        IDSetSwitch(&TemperatureCompensateOnStartSP, nullptr);

        // Temperature Coeff A
        if (isSimulation())
        {
            snprintf(response, 32, "TempCo A = %d\n", static_cast<int>(TemperatureParamN[0].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (TemperatureCompensateModeS[0].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TCoeff;
            rc = sscanf(response, "%15[^=]=%d", key, &TCoeff);
            if (rc != 2)
                return false;

            TemperatureParamN[0].value = TCoeff;
        }
        memset(response, 0, sizeof(response));

        // Temperature Coeff B
        if (isSimulation())
        {
            snprintf(response, 32, "TempCo B = %d\n", static_cast<int>(TemperatureParamN[0].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[1].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TCoeff;
            rc = sscanf(response, "%15[^=]=%d", key, &TCoeff);
            if (rc != 2)
                return false;

            TemperatureParamN[0].value = TCoeff;
        }

        memset(response, 0, sizeof(response));

        // Temperature Coeff C
        if (isSimulation())
        {
            snprintf(response, 32, "TempCo C = %d\n", static_cast<int>(TemperatureParamN[0].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[2].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TCoeff;
            rc = sscanf(response, "%15[^=]=%d", key, &TCoeff);
            if (rc != 2)
                return false;

            TemperatureParamN[0].value = TCoeff;
        }

        memset(response, 0, sizeof(response));

        // Temperature Coeff D
        if (isSimulation())
        {
            snprintf(response, 32, "TempCo D = %d\n", static_cast<int>(TemperatureParamN[0].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[3].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TCoeff;
            rc = sscanf(response, "%15[^=]=%d", key, &TCoeff);
            if (rc != 2)
                return false;

            TemperatureParamN[0].value = TCoeff;
        }

        memset(response, 0, sizeof(response));

        // Temperature Coeff E
        if (isSimulation())
        {
            snprintf(response, 32, "TempCo E = %d\n", static_cast<int>(TemperatureParamN[0].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[4].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TCoeff;
            rc = sscanf(response, "%15[^=]=%d", key, &TCoeff);
            if (rc != 2)
                return false;

            TemperatureParamN[0].value = TCoeff;
        }

        memset(response, 0, sizeof(response));

        // Temperature intercepts A
        if (isSimulation())
        {
            snprintf(response, 32, "TempIn A = %d\n", static_cast<int>(TemperatureParamN[1].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[0].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TInter;
            rc = sscanf(response, "%15[^=]=%d", key, &TInter);
            if (rc != 2)
                return false;

            TemperatureParamN[1].value = TInter;
        }

        memset(response, 0, sizeof(response));

        // Temperature intercepts B
        if (isSimulation())
        {
            snprintf(response, 32, "TempIn B = %d\n", static_cast<int>(TemperatureParamN[1].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[1].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TInter;
            rc = sscanf(response, "%15[^=]=%d", key, &TInter);
            if (rc != 2)
                return false;

            TemperatureParamN[1].value = TInter;
        }

        memset(response, 0, sizeof(response));

        // Temperature intercepts C
        if (isSimulation())
        {
            snprintf(response, 32, "TempIn C = %d\n", static_cast<int>(TemperatureParamN[1].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[2].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TInter;
            rc = sscanf(response, "%15[^=]=%d", key, &TInter);
            if (rc != 2)
                return false;

            TemperatureParamN[1].value = TInter;
        }

        memset(response, 0, sizeof(response));

        // Temperature intercepts D
        if (isSimulation())
        {
            snprintf(response, 32, "TempIn D = %d\n", static_cast<int>(TemperatureParamN[1].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[3].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TInter;
            rc = sscanf(response, "%15[^=]=%d", key, &TInter);
            if (rc != 2)
                return false;

            TemperatureParamN[1].value = TInter;
        }

        memset(response, 0, sizeof(response));

        // Temperature intercepts E
        if (isSimulation())
        {
            snprintf(response, 32, "TempIn E = %d\n", static_cast<int>(TemperatureParamN[1].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        if (TemperatureCompensateModeS[4].s == ISS_ON)
        {
            response[nbytes_read - 1] = '\0';
            LOGF_DEBUG("RES (%s)", response);

            int TInter;
            rc = sscanf(response, "%15[^=]=%d", key, &TInter);
            if (rc != 2)
                return false;

            TemperatureParamN[1].value = TInter;
        }

        TemperatureParamNP.s = IPS_OK;
        IDSetNumber(&TemperatureParamNP, nullptr);

        memset(response, 0, sizeof(response));

        // StepSize
        if (isSimulation())
        {
            snprintf(response, 32, "StepSize = %d\n", static_cast<int>(StepSizeN[0].value));
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        int valueStepSize;
        rc = sscanf(response, "%15[^=]=%d", key, &valueStepSize);
        if (rc != 2)
            return false;

        StepSizeN[0].value = valueStepSize;
        IDSetNumber(&StepSizeNP, nullptr);

        memset(response, 0, sizeof(response));

        // END is reached
        memset(response, 0, sizeof(response));
        if (isSimulation())
        {
            strncpy(response, "END\n", 16);
            nbytes_read = strlen(response);
        }
        else if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (nbytes_read > 0)
        {
            response[nbytes_read - 1] = '\0';

            // Display the response to be sure to have read the complete TTY Buffer.
            LOGF_DEBUG("RES (%s)", response);
            if (strcmp(response, "END"))
                return false;
        }

        tcflush(PortFD, TCIFLUSH);

        return true;
    }
    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setDeviceType(int index)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sSCDT%s>", getFocusTarget(), index > 0 ? lynxModels[ModelS[index].name].c_str() : "ZZ");

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setLedLevel(int level)
// Write via the connected port to the HUB the selected LED intensity level

{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<FHSCLB%d>", level);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setDeviceNickname(const char *nickname)
// Write via the connected port to the HUB the choiced nikname of the focuser
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sSCNN%s>", getFocusTarget(), nickname);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}
/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::home()
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sHOME>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "H", 16);
        nbytes_read              = strlen(response) + 1;
        targetPosition           = 0;
        //FocusAbsPosNP[0].setValue(MaxTravelN[0].value);
        FocusAbsPosNP.setState(IPS_OK);
        FocusAbsPosNP.apply();
        simStatus[STATUS_HOMING] = ISS_ON;
        simStatus[STATUS_HOMED] = ISS_OFF;
        simPosition = FocusAbsPosNP[0].getValue();
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        FocusAbsPosNP.setState(IPS_BUSY);
        FocusAbsPosNP.apply();

        isHoming = true;
        LOG_INFO("Focuser is homing...");

        tcflush(PortFD, TCIFLUSH);

        return true;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::center()
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sCENTER>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "M", 16);
        nbytes_read              = strlen(response) + 1;
        simStatus[STATUS_MOVING] = ISS_ON;
        targetPosition           = FocusAbsPosNP[0].getMax() / 2;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        LOG_INFO("Focuser moving to center position...");

        FocusAbsPosNP.setState(IPS_BUSY);
        FocusAbsPosNP.apply();

        tcflush(PortFD, TCIFLUSH);

        return true;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setTemperatureCompensation(bool enable)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sSCTE%d>", getFocusTarget(), enable ? 1 : 0);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setTemperatureCompensationMode(char mode)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sSCTM%c>", getFocusTarget(), mode);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        // If OK, the value would be read and update UI properties
        if (!strcmp(response, "SET"))
            return getFocusTemp();
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setTemperatureCompensationCoeff(char mode, int16_t coeff)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sSCTC%c%c%04d>", getFocusTarget(), mode, coeff >= 0 ? '+' : '-', static_cast<int>(std::abs(coeff)));

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setTemperatureInceptions(char mode, int32_t inter)
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sSETINT%c%c%06d>", getFocusTarget(), mode, inter >= 0 ? '+' : '-',
             static_cast<int>(std::abs(inter)));

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setTemperatureCompensationOnStart(bool enable)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sSCTS%d>", getFocusTarget(), enable ? 1 : 0);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
//bool FocusLynxBase::setBacklashCompensation(bool enable)
bool FocusLynxBase::SetFocuserBacklashEnabled(bool enabled)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sSCBE%d>", getFocusTarget(), enabled ? 1 : 0);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
//bool FocusLynxBase::setFocusBacklashSteps(uint16_t steps)
bool FocusLynxBase::SetFocuserBacklash(int32_t steps)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sSCBS%02d>", getFocusTarget(), steps);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::ReverseFocuser(bool enabled)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sREVERSE%d>", getFocusTarget(), enabled ? 1 : 0);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read               = strlen(response) + 1;
        simStatus[STATUS_REVERSE] = enabled ? ISS_ON : ISS_OFF;
    }
    else
    {
        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
            return true;
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::SyncFocuser(uint32_t ticks)
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sSCCP%06d>", getFocusTarget(), ticks);
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        simPosition = ticks;
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
        {
            LOGF_INFO("Setting current position to %d", ticks);
            return true;
        }
        else
            return false;
    }
    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
//bool FocusLynxBase::setMaxTravel(uint16_t travel)
bool FocusLynxBase::SetFocuserMaxPosition(uint32_t ticks)
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sSETMAX%06d>", getFocusTarget(), ticks);
    LOGF_DEBUG("CMD (%s)", cmd);

    SyncPresets(ticks);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
        {
            getFocusConfig();
            return true;
        }
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::setStepSize(uint16_t stepsize)
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sSETFSS%06d>", getFocusTarget(), stepsize);
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
        {
            getFocusConfig();
            return true;
        }
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::resetFactory()
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sRESET>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "SET", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);
        tcflush(PortFD, TCIFLUSH);

        if (!strcmp(response, "SET"))
        {
            getFocusConfig();
            return true;
        }
        else
            return false;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::isResponseOK()
{
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read = 0;

    memset(response, 0, sizeof(response));

    if (isSimulation())
    {
        strcpy(response, "!");
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("TTY error: %s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        if (!strcmp(response, "!"))
            return true;
        else
        {
            memset(response, 0, sizeof(response));
            while (strstr(response, "END") == nullptr)
            {
                if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
                {
                    tty_error_msg(errcode, errmsg, MAXRBUF);
                    LOGF_ERROR("TTY error: %s", errmsg);
                    return false;
                }
                response[nbytes_read - 1] = '\0';
                LOGF_ERROR("Controller error: %s", response);
            }

            return false;
        }
    }
    return true;
}

/************************************************************************************
*
* ***********************************************************************************/
IPState FocusLynxBase::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    char cmd[16];
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));

    snprintf(cmd, 16, "<%sM%cR%c>", getFocusTarget(), (dir == FOCUS_INWARD) ? 'I' : 'O', (speed == 0) ? '0' : '1');

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "M", 16);
        nbytes_read = strlen(response) + 1;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return IPS_ALERT;
        }

        if (!isResponseOK())
            return IPS_ALERT;

        gettimeofday(&focusMoveStart, nullptr);
        focusMoveRequest = duration / 1000.0;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return IPS_ALERT;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        if (duration <= getCurrentPollingPeriod())
        {
            usleep(getCurrentPollingPeriod() * 1000);
            AbortFocuser();
            return IPS_OK;
        }

        tcflush(PortFD, TCIFLUSH);

        return IPS_BUSY;
    }

    return IPS_ALERT;
}

/************************************************************************************
*
* ***********************************************************************************/
IPState FocusLynxBase::MoveAbsFocuser(uint32_t targetTicks)
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    targetPosition = targetTicks;

    memset(response, 0, sizeof(response));

    snprintf(cmd, LYNX_MAX, "<%sMA%06d>", getFocusTarget(), targetTicks);

    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "M", 16);
        nbytes_read              = strlen(response) + 1;
        simStatus[STATUS_MOVING] = ISS_ON;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return IPS_ALERT;
        }

        if (!isResponseOK())
            return IPS_ALERT;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return IPS_ALERT;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        FocusAbsPosNP.setState(IPS_BUSY);

        tcflush(PortFD, TCIFLUSH);

        return IPS_BUSY;
    }

    return IPS_ALERT;
}

/************************************************************************************
*
* ***********************************************************************************/
IPState FocusLynxBase::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    uint32_t newPosition = 0;

    if (dir == FOCUS_INWARD)
        newPosition = FocusAbsPosNP[0].getValue() - ticks;
    else
        newPosition = FocusAbsPosNP[0].getValue() + ticks;

    return MoveAbsFocuser(newPosition);
}

/************************************************************************************
*
* ***********************************************************************************/
void FocusLynxBase::TimerHit()
{
    if (!isConnected())
        return;

    if (configurationComplete == false)
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    bool statusrc = false;
    for (int i = 0; i < 2; i++)
    {
        statusrc = getFocusStatus();
        if (statusrc)
            break;
    }

    if (statusrc == false)
    {
        LOG_WARN("Unable to read focuser status....");
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    if (FocusAbsPosNP.getState() == IPS_BUSY || FocusRelPosNP.getState() == IPS_BUSY)
    {
        if (isSimulation())
        {
            if (FocusAbsPosNP[0].getValue() < targetPosition)
                simPosition += 100;
            else
                simPosition -= 100;

            simStatus[STATUS_MOVING] = ISS_ON;

            if (std::abs(static_cast<int64_t>(simPosition) - static_cast<int64_t>(targetPosition)) < 100)
            {
                FocusAbsPosNP[0].setValue(targetPosition);
                simPosition              = FocusAbsPosNP[0].getValue();
                simStatus[STATUS_MOVING] = ISS_OFF;
                StatusL[STATUS_MOVING].s = IPS_IDLE;
                if (simStatus[STATUS_HOMING] == ISS_ON)
                {
                    StatusL[STATUS_HOMED].s = IPS_OK;
                    StatusL[STATUS_HOMING].s = IPS_IDLE;
                    simStatus[STATUS_HOMING] = ISS_OFF;
                    simStatus[STATUS_HOMED] = ISS_ON;
                }
            }
            else
                StatusL[STATUS_MOVING].s = IPS_BUSY;
            IDSetLight(&StatusLP, nullptr);
        }

        if (isHoming && StatusL[STATUS_HOMED].s == IPS_OK)
        {
            isHoming = false;
            GotoSP.s = IPS_OK;
            IUResetSwitch(&GotoSP);
            GotoS[GOTO_HOME].s = ISS_ON;
            IDSetSwitch(&GotoSP, nullptr);
            FocusAbsPosNP.setState(IPS_OK);
            FocusRelPosNP.apply();
            LOG_INFO("Focuser completed home.");
            if (isSimulation())
                center();
        }
        else if (StatusL[STATUS_MOVING].s == IPS_IDLE)
        {
            FocusAbsPosNP.setState(IPS_OK);
            FocusRelPosNP.setState(IPS_OK);
            FocusAbsPosNP.apply();
            FocusRelPosNP.apply();
            if (GotoSP.s == IPS_BUSY)
            {
                IUResetSwitch(&GotoSP);
                GotoSP.s = IPS_OK;
                IDSetSwitch(&GotoSP, nullptr);
            }
            LOG_INFO("Focuser reached requested position.");
        }
        else if (StatusL[STATUS_MOVING].s == IPS_BUSY && focusMoveRequest > 0)
        {
            float remaining = calcTimeLeft(focusMoveStart, focusMoveRequest);

            if (remaining < getCurrentPollingPeriod())
            {
                sleep(remaining);
                AbortFocuser();
                focusMoveRequest = 0;
            }
        }
    }
    if (StatusL[STATUS_HOMING].s == IPS_BUSY && GotoSP.s != IPS_BUSY)
    {
        GotoSP.s = IPS_BUSY;
        IDSetSwitch(&GotoSP, nullptr);
    }

    SetTimer(getCurrentPollingPeriod());
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::AbortFocuser()
{
    char cmd[LYNX_MAX] = {0};
    int errcode = 0;
    char errmsg[MAXRBUF];
    char response[LYNX_MAX] = {0};
    int nbytes_read    = 0;
    int nbytes_written = 0;

    memset(response, 0, sizeof(response));
    snprintf(cmd, LYNX_MAX, "<%sHALT>", getFocusTarget());
    LOGF_DEBUG("CMD (%s)", cmd);

    if (isSimulation())
    {
        strncpy(response, "HALTED", 16);
        nbytes_read              = strlen(response) + 1;
        simStatus[STATUS_MOVING] = ISS_OFF;
        simStatus[STATUS_HOMING] = ISS_OFF;
    }
    else
    {
        tcflush(PortFD, TCIFLUSH);

        if ((errcode = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }

        if (!isResponseOK())
            return false;

        if ((errcode = tty_read_section(PortFD, response, 0xA, LYNXFOCUS_TIMEOUT, &nbytes_read)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            LOGF_ERROR("%s", errmsg);
            return false;
        }
    }

    if (nbytes_read > 0)
    {
        response[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES (%s)", response);

        if (FocusRelPosNP.getState() == IPS_BUSY)
        {
            FocusRelPosNP.setState(IPS_IDLE);
            FocusRelPosNP.apply();
        }

        FocusTimerNP.setState(IPS_IDLE);
        FocusAbsPosNP.setState(IPS_IDLE);
        GotoSP.s = IPS_IDLE;
        IUResetSwitch(&GotoSP);
        FocusTimerNP.apply();
        FocusAbsPosNP.apply();
        IDSetSwitch(&GotoSP, nullptr);

        tcflush(PortFD, TCIFLUSH);

        return true;
    }

    return false;
}

/************************************************************************************
 *
* ***********************************************************************************/
float FocusLynxBase::calcTimeLeft(timeval start, float req)
{
    double timesince;
    double timeleft;
    struct timeval now
    {
        0, 0
    };
    gettimeofday(&now, nullptr);

    timesince =
        static_cast<int>((now.tv_sec * 1000.0 + now.tv_usec / 1000)) - static_cast<int>((start.tv_sec * 1000.0 + start.tv_usec /
                1000));
    timesince = timesince / 1000;
    timeleft  = req - timesince;
    return timeleft;
}

/************************************************************************************
 *
* ***********************************************************************************/
bool FocusLynxBase::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);

    IUSaveConfigSwitch(fp, &ModelSP);
    IUSaveConfigSwitch(fp, &TemperatureCompensateSP);
    IUSaveConfigSwitch(fp, &TemperatureCompensateOnStartSP);
    //IUSaveConfigSwitch(fp, &ReverseSP);
    IUSaveConfigNumber(fp, &TemperatureNP);
    IUSaveConfigSwitch(fp, &TemperatureCompensateModeSP);
    //IUSaveConfigSwitch(fp, &FocusBacklashSP);
    //IUSaveConfigNumber(fp, &FocusBacklashNP);
    IUSaveConfigNumber(fp, &StepSizeNP);

    return true;
}

/************************************************************************************
*
************************************************************************************/
bool FocusLynxBase::loadConfig(bool silent, const char *property)
{
    bool result = true;

    if (property == nullptr)
    {
        // Need to know the user choice for this option not store in HUB
        result = INDI::DefaultDevice::loadConfig(silent, "Presets") && result;
        if (isSimulation())
        {
            // Only load for simulation, otherwise got from the HUB
            result = (INDI::DefaultDevice::loadConfig(silent, "MODEL") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "T. COMPENSATION") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "T. COMPENSATION @START") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "REVERSE") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "T. COEFF") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "COMPENSATE MODE") && result);
            //            result = (INDI::DefaultDevice::loadConfig(silent, "BACKLASH COMPENSATION") && result);
            //            result = (INDI::DefaultDevice::loadConfig(silent, "BACKLASH") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "MAX TRAVEL") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "STEP SIZE") && result);
            result = (INDI::DefaultDevice::loadConfig(silent, "T. PARAMETERS") && result);
        }
    }
    else
        result = INDI::DefaultDevice::loadConfig(silent, property);

    return result;
}

/************************************************************************************
 *
* ***********************************************************************************/
void FocusLynxBase::debugTriggered(bool enable)
{
    INDI_UNUSED(enable);
    //tty_set_debug(enable ? 1 : 0);
}

/************************************************************************************
 *
* ***********************************************************************************/
void FocusLynxBase::setFocusTarget(const char *target)
// Use to set the string of the private char[] focusTarget
{
    strncpy(focusTarget, target, 8);
}

/************************************************************************************
 *
* ***********************************************************************************/
const char *FocusLynxBase::getFocusTarget()
// Use to get the string of the private char[] focusTarget
{
    return focusTarget;
}

/************************************************************************************
 *
* ***********************************************************************************/
int FocusLynxBase::getVersion(int *major, int *minor, int *sub)
{
    INDI_UNUSED(major);
    INDI_UNUSED(minor);
    INDI_UNUSED(sub);
    /* For future use of implementation of new firmware 2.0.0
     * and give ability to keep compatible to actual 1.0.9
     * Will be to avoid calling to new functions
     * Not yet implemented in this version of the driver
     */
    char sMajor[8], sMinor[8], sSub[8];
    int  rc = sscanf(version, "%[^.].%[^.].%s", sMajor, sMinor, sSub);

    LOGF_DEBUG("Version major: %s, minor: %s, subversion: %s", sMajor, sMinor, sSub);
    *major = atoi(sMajor);
    *minor = atoi(sMinor);
    *sub = atoi(sSub);
    if (rc == 3)
        return *major;
    return 0;  // 0 Means error in this case
}
