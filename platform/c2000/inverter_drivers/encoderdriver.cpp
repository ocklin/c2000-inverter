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

#include "c2000/encoderdriver.h"
#include "c2000/adcdriver.h"
#include "device.h"

namespace c2000 {

// pin 160 EPWM1A	EXCITER SQ WAVE O/P 10kHz 50% duty cycle, [TP100]
// pin 43	ADCINA0	RESOLVER SINE
// pin 47	ADCINB1	RESOLVER COSINE
// pin 60	ADCIND4	EXCITER MONITOR ANALOG I/P

const AdcInputT EncoderDriverResolverSinus = {
    ADCA_BASE, ADCARESULT_BASE, ADC_CH_ADCIN0
};
const AdcInputT EncoderDriverResolverCosinus = {
    ADCB_BASE, ADCBRESULT_BASE, ADC_CH_ADCIN1
};
const AdcInputT EncoderDriverExciterMonitor = {
    ADCD_BASE, ADCDRESULT_BASE, ADC_CH_ADCIN4
};

const ADC_IntNumber EncoderDriverAdcIntNumber = ADC_INT_NUMBER2;

volatile uint32_t EncoderDriver::cycles = 0;

void EncoderDriver::Init(uint16_t pwmmax) 
{
    // Enhanced PWM1 output A (HRPWM-capable)
    uint32_t base = EPWM1_BASE; 

    //EPWM1 -> myEPWM1 Pinmux
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, pwmmax);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // Set Compare values to 50% duty cycle for exciter
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, pwmmax/2);

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Load shadow compare in the center (zero)
    //
    EPWM_setCounterCompareShadowLoadMode(
        base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);


    // configure as dependent on 4
    //
    // ePWM1 uses the ePWM 4 SYNCO as its SYNCIN.
    //
    /*    
    EPWM_setSyncOutPulseMode(base, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_enablePhaseShiftLoad(base);
    // EPWM_setPhaseShift(base, 2);
    EPWM_setCountModeAfterSync(base, EPWM_COUNT_MODE_UP_AFTER_SYNC);
    */

   InitAdc();
}

void EncoderDriver::InitAdc() {

    const ADC_SOCNumber soc = ADC_SOC_NUMBER1;

    AdcDriver::Init(EncoderDriverResolverSinus.base);
    AdcDriver::Init(EncoderDriverResolverCosinus.base);
    AdcDriver::Init(EncoderDriverExciterMonitor.base);

    // delay to allow ADCs to power up
    DEVICE_DELAY_US(1500U);

    AdcDriver::Setup(EncoderDriverResolverSinus.base, EncoderDriverResolverSinus.channel, soc, 14);
    AdcDriver::Setup(EncoderDriverResolverCosinus.base, EncoderDriverResolverCosinus.channel, soc, 14);
    AdcDriver::Setup(EncoderDriverExciterMonitor.base, EncoderDriverExciterMonitor.channel, soc, 14);

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
    ADC_setInterruptSource(EncoderDriverResolverSinus.base, EncoderDriverAdcIntNumber, soc);
    ADC_enableContinuousMode(EncoderDriverResolverSinus.base, EncoderDriverAdcIntNumber);
    ADC_enableInterrupt(EncoderDriverResolverSinus.base, EncoderDriverAdcIntNumber);
}

/*
 * get the measured sin/cos values
 */
void EncoderDriver::getSinCos(volatile int32_t &sin, volatile int32_t &cos, volatile int32_t &monitor) {

    // wait on ADC EOC
    while(ADC_getInterruptStatus(EncoderDriverResolverSinus.base, EncoderDriverAdcIntNumber) == 0) cycles++;
    NOP;    //1 cycle delay for ADC PPB result

    sin = ADC_readPPBResult(EncoderDriverResolverSinus.resultBase, ADC_PPB_NUMBER1);
    cos = ADC_readPPBResult(EncoderDriverResolverCosinus.resultBase, ADC_PPB_NUMBER1);

    monitor = ADC_readPPBResult(EncoderDriverExciterMonitor.resultBase, ADC_PPB_NUMBER1);

    ADC_clearInterruptStatus(EncoderDriverResolverSinus.base, EncoderDriverAdcIntNumber);
}

} // namespace c2000

