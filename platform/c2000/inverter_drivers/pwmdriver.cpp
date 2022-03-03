/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
 * Copyright (C) 2021 David J. Fiddes <D.J@fiddes.net>
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

#include "c2000/pwmdriver.h"
#include "device.h"
#include "driverlib.h"
#include "errormessage.h"
#include "params.h"
#include "c2000/currentvoltagedriver.h"
#include "c2000/encoderdriver.h"
#include "c2000/performancecounter.h"
#include "c2000/pwmgeneration.h"

namespace c2000 {

/** Phase A EPWM Base Address */
uint32_t PwmDriver::sm_phaseAEpwmBase;

/** Phase B EPWM Base Address */
uint32_t PwmDriver::sm_phaseBEpwmBase;

/** Phase B EPWM Base Address */
uint32_t PwmDriver::sm_phaseCEpwmBase;

uint16_t PwmDriver::pwmmax;

/**
 * Driver Initialisation
 */
void PwmDriver::DriverInit()
{
}

/**
 * Enable master PWM output
 */
void PwmDriver::EnableMasterOutput()
{
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);
}

/**
 * Disable master PWM output
 */
void PwmDriver::DisableMasterOutput()
{
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseAEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseBEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_OUTPUT_LOW);
    EPWM_setActionQualifierContSWForceAction(
        sm_phaseCEpwmBase, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);
}

/**
 * Enable timer PWM output
 */
void PwmDriver::EnableOutput()
{
    // Not used on the C2000
}

/**
 * Disable timer PWM output
 */
void PwmDriver::DisableOutput()
{
    // Not used on the C2000
}

/**
 * Set the PWM values for each phase
 */
void PwmDriver::SetPhasePwm(uint32_t phaseA, uint32_t phaseB, uint32_t phaseC)
{
    // Load in the phase values - these will be loaded from the shadow registers
    // when the counter reaches zero
    // uint32_t registerOffset = EPWM_O_CMPA + (uint16_t)EPWM_COUNTER_COMPARE_A;
    // HWREGH(sm_phaseAEpwmBase + registerOffset + 0x1U) = phaseA;

    EPWM_setCounterCompareValue(
        sm_phaseAEpwmBase, EPWM_COUNTER_COMPARE_A, phaseA);
    EPWM_setCounterCompareValue(
        sm_phaseBEpwmBase, EPWM_COUNTER_COMPARE_A, phaseB);
    EPWM_setCounterCompareValue(
        sm_phaseCEpwmBase, EPWM_COUNTER_COMPARE_A, phaseC);
}

/** Store of number of SYSCLOCK ticks we spend running the main PWM interrupt
 * handler
 */
static volatile int32_t execTicks = 0;
static volatile int32_t rounds = 0;

/**
 * Main PWM timer interrupt. Run the main motor control loop while measuring how
 * long we take
 */
__interrupt void pwm_timer_isr(void)
{
    uint32_t startTime = PerformanceCounter::GetCount();

    PwmGeneration::Run();

    // Measure the time - handles timer overflows
    execTicks += (startTime - PerformanceCounter::GetCount());
    rounds++;

    EPWM_clearEventTriggerInterruptFlag(EPWM5_BASE);

    // Acknowledge interrupt group (same for both Tesla M3 inverter and
    // LAUNCHXL-F28379D)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

/**
 * Enable the PWM outputs for charging
 * \param opmode Operating mode only BUCK and BOOST will do anything
 */
void PwmDriver::EnableChargeOutput(Modes opmode)
{
    // Not relevant to this HW
}

/**
 * Enable the PWM outputs for AC heat
 */
void PwmDriver::EnableACHeatOutput()
{
    // Not relevant to this HW
}

/**
 * Program the PWM output compare units for our over current limits
 * \param limNeg    Negative over current limit
 * \param limPos    Positive over current limit
 */
void PwmDriver::SetOverCurrentLimits(int16_t limNeg, int16_t limPos)
{
    // TODO: Implement possibly via filtered analog compare and EPWM Trip-Zone
}

/**
 * Initialise an individual EPMW module
 *
 * \param[in] base The EPWM module to configure
 * \param[in] pwmmax The PWM period
 * \param[in] deadBandCount The size of the deadband in PWM counts
 */
static void initEPWM(uint32_t base, uint16_t pwmmax, uint16_t deadBandCount)
{
    //uint16_t phaseShift = (4096U - 3280U);
    uint16_t phaseShift = 0U;

    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, pwmmax);
    EPWM_setPhaseShift(base, phaseShift);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setCountModeAfterSync(base, EPWM_COUNT_MODE_UP_AFTER_SYNC);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A, 0);

    EPWM_selectPeriodLoadEvent(base, EPWM_SHADOW_LOAD_MODE_SYNC);

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Load shadow compare in the centre (zero)
    //
    EPWM_setActionQualifierContSWForceShadowMode(
        base, EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_A,
        EPWM_AQ_OUTPUT_LOW,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(
        base,
        EPWM_AQ_OUTPUT_B,
        EPWM_AQ_OUTPUT_HIGH,
        EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Use EPWMA as the input for both RED and FED
    //
    EPWM_setRisingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(base, EPWM_DB_INPUT_EPWMA);

    //
    // Set the RED and FED values
    //
    EPWM_setFallingEdgeDelayCount(base, deadBandCount);
    EPWM_setRisingEdgeDelayCount(base, deadBandCount);

    //
    // Invert only the Falling Edge delayed output (AHC)
    //
    EPWM_setDeadBandDelayPolarity(
        base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(
        base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    //
    // Use the delayed signals instead of the original signals
    //
    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);

    //
    // DO NOT Switch Output A with Output B
    //
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandOutputSwapMode(base, EPWM_DB_OUTPUT_B, false);
}

/**
 * Setup main PWM timer
 *
 * \param[in] deadtime Deadtime between bottom and top (in nS)
 * \param[in] activeLow Set Output Polarity true=Active Low
 * \param[in] pwmdigits Number of PWM digits we are using
 * \return PWM ISR callback frequency
 */
uint16_t PwmDriver::TimerSetup(
    uint16_t deadtime,
    bool     activeLow,
    uint16_t pwmdigits)
{
    const uint32_t EPWM_FREQ = DEVICE_SYSCLK_FREQ / 2;

    // Determine the maximum counter value for PWM
    pwmmax = 1U << pwmdigits;

    // Determine the count for the required deadtime
    const uint16_t nsPerCount = 1e9 / EPWM_FREQ;
    const uint16_t deadBandCount = deadtime / nsPerCount;

    //
    // Set up GPIO pinmux for EPWM
    //
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setPinConfig(GPIO_9_EPWM5B);
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setPinConfig(GPIO_11_EPWM6B);

    if (IsTeslaM3Inverter())
    {
        GPIO_setPinConfig(GPIO_12_EPWM7A);
        GPIO_setPinConfig(GPIO_13_EPWM7B);
    }
    else
    {
        // for launchpad we also set up 4 as output
        GPIO_setPinConfig(GPIO_6_EPWM4A);
        GPIO_setPinConfig(GPIO_7_EPWM4B);
    }

    // Disable sync(Freeze clock to PWM as well).
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EncoderDriver::Init(pwmmax);

    // Enable PWM 1 as the primary sync source
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    // pwm4 is used to passover pwm1 sync out to pwm 5 and 6
    SysCtl_setSyncInputConfig(
        SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
    EPWM_setSyncOutPulseMode(EPWM4_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);

    // Store the EPWM modules used for each phase for normal operation
    sm_phaseAEpwmBase = EPWM5_BASE;
    sm_phaseCEpwmBase = EPWM6_BASE;

    if (IsTeslaM3Inverter())
    {
        sm_phaseBEpwmBase = EPWM7_BASE;
    }
    else
    {
        sm_phaseBEpwmBase = EPWM4_BASE;
    }

    //
    // Initialize EPWM for all phases
    //
    initEPWM(sm_phaseAEpwmBase, pwmmax, deadBandCount);
    initEPWM(sm_phaseBEpwmBase, pwmmax, deadBandCount);
    initEPWM(sm_phaseCEpwmBase, pwmmax, deadBandCount);

    // EPWM5 uses EPWM 4 SYNCO as its SYNCIN.
    EPWM_setSyncOutPulseMode(EPWM5_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_enablePhaseShiftLoad(EPWM5_BASE);

    // EPWM6 uses EPWM 5 SYNCO as its SYNCIN.
    EPWM_setSyncOutPulseMode(EPWM6_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_enablePhaseShiftLoad(EPWM6_BASE);

    if (IsTeslaM3Inverter())
    {
        // EPWM7 uses EPWM4 SYNCO as its SYNCIN
        // to hand over into next tripple
        SysCtl_setSyncInputConfig(
            SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM4SYNCOUT);
        EPWM_enablePhaseShiftLoad(EPWM7_BASE);
    }

    // enabling phase shift load no matter if output is generated 
    //    or if 4 is just for sync forward
    EPWM_enablePhaseShiftLoad(EPWM4_BASE);

    EncoderDriver::InitAdc();

    uint32_t triggerBase = EPWM5_BASE;

    EPWM_setADCTriggerSource(triggerBase, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);
    EPWM_setADCTriggerEventPrescale(triggerBase, EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(triggerBase, EPWM_SOC_A);

    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 15th event (to allow non-optimised code)
    
    EPWM_setInterruptSource(triggerBase, EPWM_INT_TBCTR_PERIOD);
    EPWM_setInterruptEventCount(triggerBase, 1U);
    EPWM_enableInterrupt(triggerBase);
    EPWM_clearEventTriggerInterruptFlag(triggerBase);

    Interrupt_register(INT_EPWM5, &pwm_timer_isr);

    // Enable AdcA-ADCINT1- to help verify EoC before result data read
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);

    // EncoderDriver::InitInterrupts();

    //CurrentVoltageDriver::Init();

    // Ensure we have initialised the performance counter before we start
    // getting interrupts
    PerformanceCounter::Init();

    Interrupt_enable(INT_EPWM5);

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Return the pwm frequency. Because we use the up-down count mode divide by
    // 2 * the period
    return EPWM_FREQ / (2 * pwmmax);
}

/**
 * Set up the timer for AC heat use
 */
void PwmDriver::AcHeatTimerSetup()
{
    // Not relevant to this HW
}

/**
 * Enable AC heat
 * \param ampnom    Notminal current to use for AC heat
 */
void PwmDriver::AcHeat(s32fp ampnom)
{
    // Not relevant to this HW
}

/**
 * Set the charge current target
 * \param dc    Target current
 */
void PwmDriver::SetChargeCurrent(int16_t dc)
{
    // Not relevant to this HW
}

/**
 * Obtain how many SYSCLOCK cycles we spend running the main control loop
 * \return Number of ticks
 */
int32_t PwmDriver::GetCpuLoad()
{
    return execTicks;
}

int32_t PwmDriver::GetRunRounds()
{
    return rounds;
}

/**
 * Reset the CPU load stored value for when the PWM is not running
 */
void PwmDriver::ResetCpuLoad()
{
    execTicks = 0;
}

} // namespace c2000
