/*
 * This file is part of the stm32_sine project.
 *
 * Copyright (C) 2022 David J. Fiddes <D.J@fiddes.net>
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

#include "matcherhelper.h"
#include "mockresolversample.h"
#include "resolverencoder.h"
#include "utils.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utility>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::FloatNear;
using ::testing::Ge;
using ::testing::InSequence;
using ::testing::Le;
using ::testing::Ne;
using ::testing::Return;
using ::testing::Test;

typedef encoder::ResolverEncoder<MockResolverSample> Resolver;

//! Use the same PWM frequency as found on the Tesla M3 inverter
static const uint16_t DefaultPwmFrequency =
    100000000 / (2 * 4096); // 12.207 kHz

//! Number of update cycles to wait before reporting valid readings
static const int StartupCount = 4000;

//! Frequency at which we update the resolver frequency
static const int FrequencyUpdate = 100;

//! Number of PWM updates for each frequency update
static const int FreqUpdatesPerPwm = DefaultPwmFrequency / FrequencyUpdate;

//
//! Common test fixture
//
class TestResolverEncoder : public ::testing::Test
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

//! Test the initial state of the resolver encoder
TEST(TestResolverEncoder, Initialisation)
{
    MockResolverSampleImpl sample;
    EXPECT_CALL(sample, ResolverSine).Times(0);
    EXPECT_CALL(sample, ResolverCosine).Times(0);

    Resolver::Reset();

    // Verify the initial state of the encoder - we shouldn't be going anywhere
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Teardown verify that nothing untoward was reported
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

//! Test a static position with the PWM configured and running
TEST(TestResolverEncoder, StaticPosition)
{
    MockResolverSampleImpl sample;
    EXPECT_CALL(sample, ResolverSine).Times(0);
    EXPECT_CALL(sample, ResolverCosine).Times(0);

    Resolver::Reset();

    Resolver::SetPwmFrequency(DefaultPwmFrequency);

    // Initial state verification
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Figure out a sample position
    const float    TestAngle = 123.456;  // degrees
    const int16_t  TestAmplitude = 1800; // counts
    const int16_t  SineValue = TestAmplitude * sin(TestAngle * (pi / 180));
    const int16_t  CosineValue = TestAmplitude * cos(TestAngle * (pi / 180));
    const uint16_t ComputedAngle = (TestAngle * 65536) / 360;

    EXPECT_CALL(sample, ResolverSine)
        .Times(StartupCount + 1)
        .WillRepeatedly(Return(SineValue));
    EXPECT_CALL(sample, ResolverCosine)
        .Times(StartupCount + 1)
        .WillRepeatedly(Return(CosineValue));

    // Verify that nothing further changes as we complete the inital settling
    // period and start sampling for real
    for (int i = 0; i < StartupCount + 1; i++)
    {
        // Simulate an update from the main control loop indicating new
        // sine/cosine values are available
        Resolver::UpdateRotorAngle(1);

        // Simulate a periodic update of frequency from an auxiliary control
        // loop
        if (i % FreqUpdatesPerPwm == 0)
        {
            Resolver::UpdateRotorFrequency(FrequencyUpdate);
        }

        EXPECT_EQ(Resolver::SeenNorthSignal(), true);
        // The angle is held (at 0) until the signal monitor has locked
        if (i >= encoder::ResolverSignalMonitor::DebounceSamples - 1)
        {
            EXPECT_THAT(Resolver::GetRotorAngle(), IntNear(ComputedAngle, 1));
        }
        EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
        EXPECT_EQ(Resolver::GetSpeed(), 0);
        EXPECT_EQ(Resolver::GetFullTurns(), 0);
        EXPECT_EQ(Resolver::GetRotorDirection(), 0);
    }

    // Teardown verify that nothing untoward was reported
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

//! Test a static position with insufficient amplitude
TEST(TestResolverEncoder, LowAmplitude)
{
    ErrorMessage::ResetAll();
    ErrorMessage::SetTime(1);
    Resolver::Reset();

    Resolver::SetPwmFrequency(DefaultPwmFrequency);

    // Initial state verification
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Figure out a sample position
    const float    TestAngle = 56.789;  // degrees
    const int16_t  TestAmplitude = 200; // counts
    const int16_t  SineValue = TestAmplitude * sin(TestAngle * (pi / 180));
    const int16_t  CosineValue = TestAmplitude * cos(TestAngle * (pi / 180));

    // Simulate an a low amplitude input to the resolver sine/cosine signals
    MockResolverSampleImpl sample;
    EXPECT_CALL(sample, ResolverSine)
        .Times(StartupCount + 1)
        .WillRepeatedly(Return(SineValue));
    EXPECT_CALL(sample, ResolverCosine)
        .Times(StartupCount + 1)
        .WillRepeatedly(Return(CosineValue));

    // Complete the initial sampling period plus 1 sample
    for (int i = 0; i < StartupCount + 1; i++)
    {
        // Simulate an update from the main control loop indicating new
        // sine/cosine values are available
        Resolver::UpdateRotorAngle(1);

        // Simulate a periodic update of frequency from an auxiliary control
        // loop
        if (i % FreqUpdatesPerPwm == 0)
        {
            Resolver::UpdateRotorFrequency(FrequencyUpdate);
        }
    }

    // Verify final angle
    EXPECT_EQ(Resolver::SeenNorthSignal(), true);
    EXPECT_EQ(Resolver::GetRotorAngle(), 0);
    EXPECT_EQ(Resolver::GetRotorFrequency(), 0);
    EXPECT_EQ(Resolver::GetSpeed(), 0);
    EXPECT_EQ(Resolver::GetFullTurns(), 0);
    EXPECT_EQ(Resolver::GetRotorDirection(), 0);

    // Verify that we trigger hit an error because of the low amplitude input
    EXPECT_EQ(ErrorMessage::GetLastError(), ERR_LORESAMP);
}

//! Run n update cycles with fixed sine/cosine values
static void RunCycles(MockResolverSampleImpl& sample, int16_t sine, int16_t cosine, int n)
{
    EXPECT_CALL(sample, ResolverSine).Times(n).WillRepeatedly(Return(sine));
    EXPECT_CALL(sample, ResolverCosine).Times(n).WillRepeatedly(Return(cosine));
    for (int i = 0; i < n; i++)
        Resolver::UpdateRotorAngle(1);
}

//! A healthy signal must lock whatever angle the rotor is parked at. At 45
//! degrees sin == cos, so a running min/max spread never reaches the lock level.
TEST(TestResolverEncoder, LocksWhenParkedAt45Degrees)
{
    ErrorMessage::ResetAll();
    ErrorMessage::SetTime(1);
    Resolver::Reset();

    const int16_t level = 1500 * cos(45 * (pi / 180));
    MockResolverSampleImpl sample;
    RunCycles(sample, level, level, StartupCount + 1);

    EXPECT_THAT(Resolver::GetRotorAngle(), IntNear((45 * 65536) / 360, 1));
    EXPECT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
}

//! Losing the resolver signal after a good lock must be reported and the last
//! angle held.
TEST(TestResolverEncoder, SignalLossAfterLockIsReported)
{
    ErrorMessage::ResetAll();
    ErrorMessage::SetTime(1);
    Resolver::Reset();

    const float   angle = 123.456;
    const int16_t sine = 1800 * sin(angle * (pi / 180));
    const int16_t cosine = 1800 * cos(angle * (pi / 180));

    MockResolverSampleImpl sample;
    RunCycles(sample, sine, cosine, StartupCount + 1);
    ASSERT_EQ(ErrorMessage::GetLastError(), ERROR_NONE);
    const uint16_t lockedAngle = Resolver::GetRotorAngle();

    // Resolver unplugged: both channels read zero
    RunCycles(sample, 0, 0, 100);

    EXPECT_EQ(ErrorMessage::GetLastError(), ERR_LORESAMP);
    EXPECT_EQ(Resolver::GetRotorAngle(), lockedAngle);
}

//! Common setup for the rotation tests
static void ResetResolver()
{
    ErrorMessage::ResetAll();
    ErrorMessage::SetTime(1);
    Param::LoadDefaults();
    Resolver::Reset();
    Resolver::SetPwmFrequency(DefaultPwmFrequency);
}

//! Feed the resolver a signal whose angle starts at startDeg and moves by
//! degPerSample each update. DecodeAngle() reads sine first, so the phase
//! advances after each cosine read.
static void RotateResolver(MockResolverSampleImpl& sample, float startDeg,
                           float degPerSample, int samples,
                           int16_t amplitude = 1800)
{
    double phase = startDeg;
    EXPECT_CALL(sample, ResolverSine)
        .Times(samples)
        .WillRepeatedly(::testing::Invoke([&phase, amplitude]() {
            return (int16_t)lround(amplitude * sin(phase * (pi / 180)));
        }));
    EXPECT_CALL(sample, ResolverCosine)
        .Times(samples)
        .WillRepeatedly(::testing::Invoke([&phase, amplitude, degPerSample]() {
            int16_t value = (int16_t)lround(amplitude * cos(phase * (pi / 180)));
            phase += degPerSample;
            return value;
        }));
    for (int i = 0; i < samples; i++)
        Resolver::UpdateRotorAngle(1);
}

TEST(TestResolverEncoder, NegativeHalfAngleMapsToUpperRange)
{
    ResetResolver();
    MockResolverSampleImpl sample;
    RotateResolver(sample, 250, 0, StartupCount + 1);
    // One ADC count at amplitude 1800 is about 1/1800 rad, ~5.8 angle counts
    EXPECT_THAT(Resolver::GetRotorAngle(), IntNear((250 * 65536) / 360, 8));
}

TEST(TestResolverEncoder, FullTurnsCountedPerMechanicalTurn)
{
    ResetResolver();
    Param::SetInt(Param::respolepairs, 1);
    MockResolverSampleImpl sample;
    RotateResolver(sample, 10, 3, 5 * 120);
    EXPECT_EQ(Resolver::GetFullTurns(), 5u);
}

TEST(TestResolverEncoder, FullTurnsHonourResolverPolePairs)
{
    ResetResolver();
    Param::SetInt(Param::respolepairs, 2);
    MockResolverSampleImpl sample;
    RotateResolver(sample, 10, 3, 6 * 120);
    EXPECT_EQ(Resolver::GetFullTurns(), 3u);
}

TEST(TestResolverEncoder, FullTurnsCountedInBothDirections)
{
    ResetResolver();
    Param::SetInt(Param::respolepairs, 1);
    MockResolverSampleImpl sample;
    // Start at 90 so the first zero crossing comes after the 16-sample lock-in
    RotateResolver(sample, 90, -3, 4 * 120);
    EXPECT_EQ(Resolver::GetFullTurns(), 4u);
}

TEST(TestResolverEncoder, NoFullTurnWithoutCrossingZero)
{
    ResetResolver();
    Param::SetInt(Param::respolepairs, 1);
    MockResolverSampleImpl sample;
    // Swing 120..240 degrees: crosses 180 but never 0/360
    double phase = 180;
    double step = 3;
    EXPECT_CALL(sample, ResolverSine).Times(2000).WillRepeatedly(::testing::Invoke([&]() {
        return (int16_t)lround(1800 * sin(phase * (pi / 180)));
    }));
    EXPECT_CALL(sample, ResolverCosine).Times(2000).WillRepeatedly(::testing::Invoke([&]() {
        int16_t value = (int16_t)lround(1800 * cos(phase * (pi / 180)));
        if (phase + step > 240 || phase + step < 120)
            step = -step;
        phase += step;
        return value;
    }));
    for (int i = 0; i < 2000; i++)
        Resolver::UpdateRotorAngle(1);
    EXPECT_EQ(Resolver::GetFullTurns(), 0u);
}
