/*
 * This file is part of the stm32_sine project.
 *
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

#include "errormessage.h"
#include "focpwmgeneration.h"
#include "matcherhelper.h"
#include "mockcurrent.h"
#include "mockencoder.h"
#include "mockpwmdriver.h"
#include <gtest/gtest.h>

using PwmGeneration = FocPwmGeneration<MockCurrent, MockEncoder, MockPwmDriver>;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::FloatNear;
using ::testing::Ge;
using ::testing::InSequence;
using ::testing::Le;
using ::testing::Ne;
using ::testing::Return;
using ::testing::Test;

void Param::Change([[maybe_unused]] Param::PARAM_NUM paramNum)
{
}
static const uint16_t DefaultPwmFrequency = 8789; // Hz
/**
 * Common test fixture
 */
class TestFocPwmGeneration : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ErrorMessage::ResetAll();
        ErrorMessage::SetTime(1);
        EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
        Param::LoadDefaults();
    }
};

TEST_F(TestFocPwmGeneration, NormalModeRun)
{
    MockPwmDriverImpl pwmDriver;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits);
    PwmGeneration::SetCurrentOffset(2048, 2048);

    {
        InSequence seq;

        EXPECT_CALL(pwmDriver, TimerSetup)
            .WillOnce(Return(DefaultPwmFrequency));
        EXPECT_CALL(pwmDriver, DriverInit);
        EXPECT_CALL(pwmDriver, EnableOutput);
    }

    // Set up an encoder that is going forwards
    MockEncoderImpl encoder;
    EXPECT_CALL(encoder, SetPwmFrequency(DefaultPwmFrequency));

    PwmGeneration::SetOpmode(Modes::RUN);

    // Ensure the system thinks we should be going forwards
    Param::SetInt(Param::dir, 1);

    // Half-throttle accelerate I guess
    PwmGeneration::SetTorquePercent(FP_FROMFLT(50));

    // Provide some neutral values for the phase currents
    MockCurrent::SetPhase1(2048);
    MockCurrent::SetPhase2(2048);

    // We need the pole pair ratio set to correctly calculate the rotation
    // frequency
    PwmGeneration::SetPolePairRatio(1);

    // initialise the controller gains from the default parameters
    PwmGeneration::SetControllerGains(
        Param::GetInt(Param::curkp),
        Param::GetInt(Param::curki),
        Param::GetInt(Param::fwkp));

    // We need to wait for a certain number of Run() cycles before the system is
    // ready to run
    static const int StartupWait = 0xffff;

    EXPECT_CALL(pwmDriver, SetOverCurrentLimits).Times(AtLeast(2));
    EXPECT_CALL(encoder, SeenNorthSignal)
        .Times(StartupWait)
        .WillRepeatedly(Return(true));
/*
    EXPECT_CALL(encoder, GetRotorFrequency)
        .Times(StartupWait)
        .WillRepeatedly(Return(10)); 
    EXPECT_CALL(encoder, UpdateRotorAngle(1)).Times(StartupWait);
    EXPECT_CALL(encoder, GetRotorAngle)
        .Times(StartupWait)
        .WillRepeatedly(Return(20000));
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(StartupWait);
    EXPECT_CALL(pwmDriver, EnableMasterOutput).Times(0); 
    */
    EXPECT_CALL(pwmDriver, SetPhasePwm).Times(StartupWait);
  

    for (int i = 0; i < StartupWait; i++)
    {
        PwmGeneration::Run();
    }

    // Now we should be through the startup wait we can verify that PWM is
    // actually output
    EXPECT_CALL(encoder, SeenNorthSignal).WillRepeatedly(Return(true));
    EXPECT_CALL(encoder, GetRotorFrequency).WillRepeatedly(Return(10));
    EXPECT_CALL(encoder, UpdateRotorAngle(1));
    EXPECT_CALL(encoder, GetRotorAngle).WillOnce(Return(20000));
    EXPECT_CALL(pwmDriver, DisableMasterOutput).Times(0);
    EXPECT_CALL(pwmDriver, EnableMasterOutput);
    EXPECT_CALL(pwmDriver, SetPhasePwm(Ne(2048), Ne(2048), Ne(2048)));

    PwmGeneration::Run();

    // Check that we turn off cleanly
    EXPECT_CALL(pwmDriver, ResetCpuLoad);
    EXPECT_CALL(pwmDriver, DisableOutput);
    PwmGeneration::SetOpmode(Modes::OFF);

    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}
