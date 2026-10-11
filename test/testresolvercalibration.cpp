// SPDX-License-Identifier: GPL-3.0-or-later
#include "resolvercalibration.h"
#include <gtest/gtest.h>
#include <math.h>

using encoder::CorrectResolverSample;
using encoder::ResolverCalibrator;

static const uint32_t Pwm = 12207;

TEST(ResolverCalibration, CorrectionCentresOnOffset)
{
    EXPECT_EQ(CorrectResolverSample(2548, 2048, 100), 500);
    EXPECT_EQ(CorrectResolverSample(1548, 2048, 100), -500);
}

TEST(ResolverCalibration, CorrectionAppliesGain)
{
    EXPECT_EQ(CorrectResolverSample(2448, 2048, 125), 500);
}

TEST(ResolverCalibration, CorrectionSaturatesToInt16)
{
    EXPECT_EQ(CorrectResolverSample(4095, 0, 200), 8190);
    EXPECT_EQ(CorrectResolverSample(0, 4095, 200), -8190);
    EXPECT_EQ(CorrectResolverSample(65535, 0, 100), 32767);
    EXPECT_EQ(CorrectResolverSample(65535, 0, 200), 32767);
}

TEST(ResolverCalibration, CorrectionFallsBackToUnityGainOutOfRange)
{
    const int32_t gains[] = { 0, 49, 201, -100 };
    for (int32_t gain : gains)
        EXPECT_EQ(CorrectResolverSample(2548, 2048, gain), 500) << "gain " << gain;
}

//! Feed seconds of a resolver turning at turnsPerSecond electrical turns,
//! starting at startDeg and sweeping sweepDeg each turn (360 = full turns)
static void Feed(ResolverCalibrator& cal, double seconds, int32_t sinCentre,
                 int32_t sinAmp, int32_t cosCentre, int32_t cosAmp,
                 double turnsPerSecond = 2, double sweepDeg = 360,
                 double startDeg = 0)
{
    const uint32_t samples = (uint32_t)(seconds * Pwm);
    for (uint32_t i = 0; i < samples; i++)
    {
        double t = (double)i / Pwm;
        double deg = sweepDeg >= 360 ? startDeg + 360 * turnsPerSecond * t
                                     : startDeg + sweepDeg * sin(2 * M_PI * turnsPerSecond * t);
        double rad = deg * M_PI / 180;
        cal.Update((uint16_t)lround(sinCentre + sinAmp * sin(rad)),
                   (uint16_t)lround(cosCentre + cosAmp * cos(rad)));
    }
}

TEST(ResolverCalibration, CalibratorIdleIgnoresSamples)
{
    ResolverCalibrator cal;
    Feed(cal, 3, 2000, 800, 2100, 640);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Idle);
}

TEST(ResolverCalibration, CalibratorFindsOffsetsAndGain)
{
    ResolverCalibrator cal;
    cal.Start(Pwm);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Running);
    Feed(cal, 3, 2000, 800, 2100, 640);
    ASSERT_EQ(cal.GetState(), ResolverCalibrator::Done);
    EXPECT_NEAR(cal.SinOffset(), 2000, 2);
    EXPECT_NEAR(cal.CosOffset(), 2100, 2);
    EXPECT_NEAR(cal.CosGainPercent(), 125, 1);
}

TEST(ResolverCalibration, CalibratorWaitsForMinimumTime)
{
    ResolverCalibrator cal;
    cal.Start(Pwm);
    Feed(cal, 1, 2000, 800, 2100, 640);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Running);
}

TEST(ResolverCalibration, CalibratorFailsWithoutRotation)
{
    ResolverCalibrator cal;
    cal.Start(Pwm);
    Feed(cal, 30, 2000, 800, 2100, 640, 0);
    cal.Update(2000, 2740);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Failed);
}

TEST(ResolverCalibration, CalibratorFailsOnPartialSwing)
{
    ResolverCalibrator cal;
    cal.Start(Pwm);
    Feed(cal, 15, 2000, 800, 2100, 640, 1, 20);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Running);
    Feed(cal, 15.1, 2000, 800, 2100, 640, 1, 20);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Failed);
}

TEST(ResolverCalibration, CalibratorFailsOnUnbalancedChannels)
{
    // Weak cosine: spread 600 never qualifies
    ResolverCalibrator weak;
    weak.Start(Pwm);
    Feed(weak, 29, 2000, 800, 2100, 300);
    EXPECT_EQ(weak.GetState(), ResolverCalibrator::Running);
    Feed(weak, 1.1, 2000, 800, 2100, 300);
    EXPECT_EQ(weak.GetState(), ResolverCalibrator::Failed);

    // Strong cosine: gain 1600 / 3400 = 47 % is out of range
    ResolverCalibrator strong;
    strong.Start(Pwm);
    Feed(strong, 1, 2000, 800, 2100, 1700);
    EXPECT_EQ(strong.GetState(), ResolverCalibrator::Running);
    Feed(strong, 2, 2000, 800, 2100, 1700);
    EXPECT_EQ(strong.GetState(), ResolverCalibrator::Failed);
}

TEST(ResolverCalibration, CalibratorRestartClearsPreviousResult)
{
    ResolverCalibrator cal;
    cal.Start(Pwm);
    Feed(cal, 3, 2000, 800, 2100, 640);
    ASSERT_EQ(cal.GetState(), ResolverCalibrator::Done);
    cal.Start(Pwm);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Running);
    // A parked rotor must not reuse the earlier min/max
    Feed(cal, 30.1, 2000, 800, 2100, 640, 0);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Failed);
}

TEST(ResolverCalibration, CalibratorStartWithZeroFrequencyFails)
{
    ResolverCalibrator cal;
    cal.Start(0);
    EXPECT_EQ(cal.GetState(), ResolverCalibrator::Failed);
}
