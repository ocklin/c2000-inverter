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

#include "c2000/adcdriver.h"

#include "device.h"
#include "driverlib.h"

namespace c2000 {

void AdcDriver::Init(uint32_t base)
{
    // Set 12-bit single ended conversion mode
    ADC_setMode(base, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    // Set main clock scaling factor (100MHz max clock for the ADC module)
    ADC_setPrescaler(base, ADC_CLK_DIV_4_0);

    // set the ADC interrupt pulse generation to end of conversion
    ADC_setInterruptPulseMode(base, ADC_PULSE_END_OF_CONV);

    // enable the ADC
    ADC_enableConverter(base);

    // set priority of SOCs
    ADC_setSOCPriority(base, ADC_PRI_ALL_HIPRI);
}

void AdcDriver::Setup(uint32_t base, uint16_t channelNumber, ADC_SOCNumber soc, uint32_t sampleWindow)
{
    ADC_setupSOC(base, soc,
                  ADC_TRIGGER_EPWM4_SOCA, (ADC_Channel)channelNumber, sampleWindow);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(base, ADC_PPB_NUMBER1, soc);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(base, ADC_PPB_NUMBER1, 0);
}

} // c2000 namespace
