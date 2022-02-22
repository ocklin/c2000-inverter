/*
 * This file is part of the Tesla M3 OSS Inverter project.
 *
 * Copyright (C) 2022 Bernd Ocklin <bernd@ocklin.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ADCDRIVER_H
#define ADCDRIVER_H

#include <stdint.h>
#include <driverlib.h>

namespace c2000 {

// these are actually more like helper functions
typedef struct {
    uint32_t base;
    uint32_t resultBase;
    ADC_Channel channel;
} AdcInputT;

class AdcDriver
{
public:
    static void Init(uint32_t base);
    static void Setup(uint32_t base, uint16_t channelNumber, ADC_SOCNumber soc, uint32_t sampleWindow);
};

} // namespace c2000

#endif // ADCDRIVER_H
