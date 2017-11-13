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
#ifndef __NEURON_HWM__
#define __NEURON_HWM__

#include <string>
#include "linux/EApiOs.h"
#include "EApi.h"
#include "semaeapi.h"

#define HWM_ITEM_COUNT (9)

class NeuronHwm
{
    public:
        static bool IsAvailable();
        static void InitLib();
        static void UnInitLib();

        explicit NeuronHwm() : count_(0) {};
        virtual ~NeuronHwm() {};

        bool GetNextItemMessage(std::string& msg);

        enum HwmType { Temp = 0, Volt, FanSpeed };

        struct HwmItem
        {
            HwmType type;
            uint32_t id;
            std::string name;
        };

    private:
        static bool isAvailable_;
        static uint32_t libHandle_;
        static const HwmItem HwmItems[];
        static const uint32_t HwmItemCount_;

        bool createItemDataString(const HwmItem * item, uint32_t data, std::string& msg);

        uint8_t count_;

};


#endif /* __NEURON_HWM__ */