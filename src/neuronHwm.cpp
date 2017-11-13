// Copyright 2017 ADLINK Technology, Inc.
// Developer: Alan Chen (alan.chen@adlinktech.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "neuronHwm.hpp"

/* * * * * * * * * * * * * *  
 * Static Private Members  *
 * * * * * * * * * * * * * */
bool NeuronHwm::isAvailable_ = false;

uint32_t NeuronHwm::libHandle_ = 0;

const NeuronHwm::HwmItem NeuronHwm::HwmItems[] = 
{
    {NeuronHwm::Temp,         EAPI_ID_HWMON_CPU_TEMP,         "CPU"},
    {NeuronHwm::Temp,         EAPI_ID_HWMON_SYSTEM_TEMP,      "System"},
    {NeuronHwm::Volt,         EAPI_ID_HWMON_VOLTAGE_VCORE,    "Vcore"},
    {NeuronHwm::Volt,         EAPI_ID_HWMON_VOLTAGE_3V3,      "3.3V"},
    {NeuronHwm::Volt,         EAPI_ID_HWMON_VOLTAGE_VBAT,     "VBAT"},
    {NeuronHwm::Volt,         EAPI_ID_HWMON_VOLTAGE_5V,       "5V"},
    {NeuronHwm::Volt,         EAPI_ID_HWMON_VOLTAGE_5VSB,     "5VSB"},
    {NeuronHwm::FanSpeed,     EAPI_ID_HWMON_FAN_CPU,          "CPU"},
    {NeuronHwm::FanSpeed,     EAPI_ID_HWMON_FAN_SYSTEM,       "System"}
};

const uint32_t NeuronHwm::HwmItemCount_ = sizeof(HwmItems) / sizeof(HwmItems[0]);


/* * * * * * * * * * * * * 
 * Static Public Methods *
 * * * * * * * * * * * * */
bool NeuronHwm::IsAvailable() 
{
    return isAvailable_;
}


void NeuronHwm::InitLib()
{
    // Initialize the the SEMA library if it hasn't been initialized.
    if (isAvailable_ == false)
    {
        uint32_t ret = 0;
        char addr[16] = "127.0.0.1";

        ret = SemaEApiLibInitialize(false, IP_V4, addr, 0, (char *)"123", &libHandle_);
        isAvailable_ = (ret == EAPI_STATUS_SUCCESS)?  true : false;
        if (isAvailable_ == false)
        {
            printf("[ERROR] neuronHwm - Can't initialize SEMA Lib. Error code: %X\n", ret);
        }
    }
}


void NeuronHwm::UnInitLib()
{
    // Uninitialize the SEMA library if it has been initialized
    if (isAvailable_ == true)
    {
        SemaEApiLibUnInitialize(libHandle_);
        isAvailable_ = false;
    }
}


/* * * * * * * * * * * * * * 
 * Static Private Methods  *
 * * * * * * * * * * * * * */
bool NeuronHwm::createItemDataString(const HwmItem * item, uint32_t data, std::string& msg)
{
    switch (item->type)
    {
        case Temp:
            msg = "Temp " + item->name + ": " + std::to_string((data - 2731) / 10.0) + " Celcius";
            return true;

        case Volt:
            msg = "Volt " + item->name + ": " + std::to_string((data /1000.0)) + " V";
            return true;

        case FanSpeed:
            msg = "FanSpeed " + item->name + ": " + std::to_string(data) + " RPM";
            return true;

        default:
            msg = "";
            printf("[ERROR] neuronHwm - getItemDataString() failed.\n");
            return false;
    }
}


/* * * * * * * * * * 
 * Public Methods  *
 * * * * * * * * * */
bool NeuronHwm::GetNextItemMessage(std::string& msg)
{
    uint32_t data = 0, ret = 0;

    while (count_ >= HwmItemCount_)
    {
        count_ -= HwmItemCount_;
    }
    
    const HwmItem * item = &HwmItems[count_];
    count_++;

    ret = SemaEApiBoardGetValue(libHandle_ , item->id, &data);
    if (ret != EAPI_STATUS_SUCCESS)
    {
        printf("[ERROR] neuronHwm - SemaEApiBoardGetValue() Failed.\n");
        return false;        
    }
    
    return createItemDataString(item, data, msg);
}