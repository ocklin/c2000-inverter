// SPDX-License-Identifier: GPL-3.0-or-later
#include "resolversignalmonitor.h"
#include <gtest/gtest.h>
#include <math.h>

using encoder::ResolverSignalMonitor;

static const int Debounce = ResolverSignalMonitor::DebounceSamples;

static void Feed(ResolverSignalMonitor& mon, int16_t sin, int16_t cos, int count)
{
    for (int i = 0; i < count; ++i)
        mon.Update(sin, cos);
}

static void FeedAngle(ResolverSignalMonitor& mon, double deg, double amplitude, int count)
{
    const double rad = deg * M_PI / 180.0;
    Feed(mon, (int16_t)(amplitude * sin(rad)), (int16_t)(amplitude * cos(rad)), count);
}

TEST(ResolverSignalMonitor, StartsUnlocked)
{
    ResolverSignalMonitor mon;
    EXPECT_FALSE(mon.Locked());
}

TEST(ResolverSignalMonitor, LocksAtAnyParkedAngle)
{
    // Includes 45 degrees, where sin == cos and a min/max spread stays small.
    for (double deg = 0; deg < 360; deg += 22.5)
    {
        ResolverSignalMonitor mon;
        FeedAngle(mon, deg, 1500, Debounce);
        EXPECT_TRUE(mon.Locked()) << "angle " << deg;
    }
}

TEST(ResolverSignalMonitor, NeedsConsecutiveGoodSamplesToLock)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 30, 1500, Debounce - 1);
    EXPECT_FALSE(mon.Locked());
    FeedAngle(mon, 30, 1500, 1);
    EXPECT_TRUE(mon.Locked());
}

TEST(ResolverSignalMonitor, LowAmplitudeNeverLocks)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 56.789, 200, 10 * Debounce);
    EXPECT_FALSE(mon.Locked());
}

TEST(ResolverSignalMonitor, LosesLockWhenSignalDisappears)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 123, 1500, Debounce);
    ASSERT_TRUE(mon.Locked());
    Feed(mon, 0, 0, Debounce - 1);
    EXPECT_TRUE(mon.Locked());
    Feed(mon, 0, 0, 1);
    EXPECT_FALSE(mon.Locked());
}

TEST(ResolverSignalMonitor, LosesLockWhenParkedAtAnAngleThatOldCheckMissed)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 45, 1500, Debounce);
    ASSERT_TRUE(mon.Locked());
    FeedAngle(mon, 45, 100, Debounce);
    EXPECT_FALSE(mon.Locked());
}

TEST(ResolverSignalMonitor, SingleBadSampleWhileLockedIsForgiven)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 80, 1500, Debounce);
    for (int i = 0; i < 100; ++i)
    {
        Feed(mon, 0, 0, Debounce - 1);
        FeedAngle(mon, 80, 1500, 1);
    }
    EXPECT_TRUE(mon.Locked());
}

TEST(ResolverSignalMonitor, SingleGoodSampleWhileUnlockedDoesNotRelock)
{
    ResolverSignalMonitor mon;
    for (int i = 0; i < 100; ++i)
    {
        FeedAngle(mon, 80, 1500, Debounce - 1);
        Feed(mon, 0, 0, 1);
    }
    EXPECT_FALSE(mon.Locked());
}

TEST(ResolverSignalMonitor, HysteresisBetweenUnlockAndLockLevels)
{
    // 500 counts lies between the unlock and lock radii.
    ResolverSignalMonitor locked;
    FeedAngle(locked, 10, 1500, Debounce);
    FeedAngle(locked, 10, 500, 10 * Debounce);
    EXPECT_TRUE(locked.Locked());

    ResolverSignalMonitor unlocked;
    FeedAngle(unlocked, 10, 500, 10 * Debounce);
    EXPECT_FALSE(unlocked.Locked());
}

TEST(ResolverSignalMonitor, ExtremeSamplesDoNotOverflow)
{
    ResolverSignalMonitor mon;
    Feed(mon, 32767, 32767, Debounce);
    EXPECT_TRUE(mon.Locked());
    Feed(mon, -32768, -32768, Debounce);
    EXPECT_TRUE(mon.Locked());
}

TEST(ResolverSignalMonitor, ResetUnlocks)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 200, 1500, Debounce);
    ASSERT_TRUE(mon.Locked());
    mon.Reset();
    EXPECT_FALSE(mon.Locked());
}

TEST(ResolverSignalMonitor, SampleBelowUnlockLevelIsNotUsableEvenWhileStillLocked)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 40, 1500, Debounce);
    ASSERT_TRUE(mon.Locked());
    EXPECT_TRUE(mon.SampleUsable());
    Feed(mon, 0, 0, 1);
    EXPECT_TRUE(mon.Locked());
    EXPECT_FALSE(mon.SampleUsable());
    FeedAngle(mon, 40, 1500, 1);
    EXPECT_TRUE(mon.SampleUsable());
}

TEST(ResolverSignalMonitor, SampleNotUsableBeforeLock)
{
    ResolverSignalMonitor mon;
    FeedAngle(mon, 40, 1500, Debounce - 1);
    EXPECT_FALSE(mon.Locked());
    EXPECT_FALSE(mon.SampleUsable());
}
