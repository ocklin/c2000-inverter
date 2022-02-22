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

#include "c2000/currentvoltagedriver.h"
#include "c2000/adcdriver.h"
#include "device.h"

// pin 39	ADCINA4	PHASE A CURRENT SENSOR
// pin 31	ADCINC2	PHASE B CURRENT SENSOR
// pin 48	ADCINB2	DC LINK VOLTAGE

namespace c2000 {

volatile int32_t CurrentVoltageDriver::measureACurrent;
volatile int32_t CurrentVoltageDriver::measureBCurrent;
volatile int32_t CurrentVoltageDriver::measureDCV;

volatile uint32_t CurrentVoltageDriver::cycles = 0;

const AdcInputT CurrentVoltageDriverPhaseACurrent = {
    ADCA_BASE, ADCARESULT_BASE, ADC_CH_ADCIN4
};
const AdcInputT CurrentVoltageDriverPhaseBCurrent = {
    ADCC_BASE, ADCCRESULT_BASE, ADC_CH_ADCIN2
};
const AdcInputT CurrentVoltageDriverHVDC = {
    ADCB_BASE, ADCBRESULT_BASE, ADC_CH_ADCIN2
};

const int32_t ADC_PU_SCALE_FACTOR = 1;
const int32_t VOLTAGE_SF = 1;


void CurrentVoltageDriver::Init() {

    AdcDriver::Init(CurrentVoltageDriverPhaseACurrent.base);
    AdcDriver::Init(CurrentVoltageDriverPhaseBCurrent.base);
    AdcDriver::Init(CurrentVoltageDriverHVDC.base);

    // delay to allow ADCs to power up
    DEVICE_DELAY_US(1500U);

    AdcDriver::Setup(CurrentVoltageDriverPhaseACurrent.base, CurrentVoltageDriverPhaseACurrent.channel, ADC_SOC_NUMBER0, 14);
    AdcDriver::Setup(CurrentVoltageDriverPhaseBCurrent.base, CurrentVoltageDriverPhaseBCurrent.channel, ADC_SOC_NUMBER0, 14);
    AdcDriver::Setup(CurrentVoltageDriverHVDC.base, CurrentVoltageDriverHVDC.channel, ADC_SOC_NUMBER0, 14);

    // Select SOC from counter, setting some irq base as for motor control loop
    // TODO - there must be a more elegant way, at least via a common define

    // TODO - this requires more investigation,
    //        xxx_PERIOD causes EOC wait loop in engine irq to spend far too much time
    EPWM_setADCTriggerSource(EPWM4_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    // Generate pulse on 1st event
    EPWM_setADCTriggerEventPrescale(EPWM4_BASE, EPWM_SOC_A, 1);
    // Enable SOC on A group
    EPWM_enableADCTrigger(EPWM4_BASE, EPWM_SOC_A);

    // Enable AdcA-ADCINT1- to help verify EoC before result data read
    ADC_setInterruptSource(CurrentVoltageDriverPhaseACurrent.base, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableContinuousMode(CurrentVoltageDriverPhaseACurrent.base, ADC_INT_NUMBER1);
    ADC_enableInterrupt(CurrentVoltageDriverPhaseACurrent.base, ADC_INT_NUMBER1);
}

/*
 * get the current as measured at engine
 */
void CurrentVoltageDriver::getMeasurement() {

    // wait on ADC EOC
    while(ADC_getInterruptStatus(CurrentVoltageDriverPhaseACurrent.base, ADC_INT_NUMBER1) == 0) {cycles++;};
    NOP;    //1 cycle delay for ADC PPB result

    measureACurrent = ADC_readPPBResult(CurrentVoltageDriverPhaseACurrent.resultBase, ADC_PPB_NUMBER1) * ADC_PU_SCALE_FACTOR;
    measureBCurrent = ADC_readPPBResult(CurrentVoltageDriverPhaseBCurrent.resultBase, ADC_PPB_NUMBER1) * ADC_PU_SCALE_FACTOR;

    int32_t vcd = ADC_readPPBResult(CurrentVoltageDriverHVDC.resultBase, ADC_PPB_NUMBER1);
    measureDCV =  vcd * VOLTAGE_SF;

    ADC_clearInterruptStatus(CurrentVoltageDriverPhaseACurrent.base, ADC_INT_NUMBER1);
}


} // namespace c2000
