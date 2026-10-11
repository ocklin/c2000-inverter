// SPDX-License-Identifier: GPL-3.0-or-later
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "dclinktrip.h"
#include <assert.h>
#include <stdio.h>

static const s32fp Min = FP_FROMINT(40);
static const s32fp Max = FP_FROMINT(60);
static const uint16_t UnderSamples = 4;

static DcLinkTrip Make()
{
    DcLinkTrip trip;
    trip.Configure(UnderSamples);
    return trip;
}

static void InRangeDoesNotTrip()
{
    DcLinkTrip trip = Make();
    for (int i = 0; i < 100; ++i)
        trip.Update(FP_FROMINT(53), Min, Max, true);
    assert(trip.State() == DcLinkTrip::OK);
}

static void OverVoltageTripsOnFirstSample()
{
    DcLinkTrip trip = Make();
    trip.Update(FP_FROMFLT(60.1), Min, Max, true);
    assert(trip.State() == DcLinkTrip::OVER);
}

static void OverVoltageTripsEvenWhenNotRunning()
{
    // The charged DC link can exceed the limit with the output off.
    DcLinkTrip trip = Make();
    trip.Update(FP_FROMINT(70), Min, Max, false);
    assert(trip.State() == DcLinkTrip::OVER);
}

static void LimitsThemselvesAreAllowed()
{
    DcLinkTrip trip = Make();
    for (int i = 0; i < 10; ++i)
    {
        trip.Update(Max, Min, Max, true);
        trip.Update(Min, Min, Max, true);
    }
    assert(trip.State() == DcLinkTrip::OK);
}

static void UnderVoltageNeedsConsecutiveSamples()
{
    DcLinkTrip trip = Make();
    for (uint16_t i = 1; i < UnderSamples; ++i)
    {
        trip.Update(FP_FROMINT(30), Min, Max, true);
        assert(trip.State() == DcLinkTrip::OK);
    }
    trip.Update(FP_FROMINT(30), Min, Max, true);
    assert(trip.State() == DcLinkTrip::UNDER);
}

static void UnderVoltageGlitchIsForgiven()
{
    DcLinkTrip trip = Make();
    for (int i = 0; i < 50; ++i)
    {
        trip.Update(FP_FROMINT(30), Min, Max, true);
        trip.Update(FP_FROMINT(30), Min, Max, true);
        trip.Update(FP_FROMINT(53), Min, Max, true);
    }
    assert(trip.State() == DcLinkTrip::OK);
}

static void UnderVoltageIgnoredWhileNotRunning()
{
    DcLinkTrip trip = Make();
    for (int i = 0; i < 100; ++i)
        trip.Update(0, Min, Max, false);
    assert(trip.State() == DcLinkTrip::OK);
}

static void UnderVoltageCountRestartsAfterOutputWasOff()
{
    DcLinkTrip trip = Make();
    for (uint16_t i = 0; i < UnderSamples - 1; ++i)
        trip.Update(FP_FROMINT(30), Min, Max, true);
    trip.Update(FP_FROMINT(30), Min, Max, false);
    trip.Update(FP_FROMINT(30), Min, Max, true);
    assert(trip.State() == DcLinkTrip::OK);
}

static void TripIsLatchedUntilReset()
{
    DcLinkTrip trip = Make();
    trip.Update(FP_FROMINT(70), Min, Max, true);
    trip.Update(FP_FROMINT(53), Min, Max, true);
    assert(trip.State() == DcLinkTrip::OVER);
    trip.Reset();
    assert(trip.State() == DcLinkTrip::OK);
    trip.Update(FP_FROMINT(53), Min, Max, true);
    assert(trip.State() == DcLinkTrip::OK);
}

static void FirstFaultIsKept()
{
    DcLinkTrip trip = Make();
    trip.Update(FP_FROMINT(70), Min, Max, true);
    for (int i = 0; i < 10; ++i)
        trip.Update(FP_FROMINT(10), Min, Max, true);
    assert(trip.State() == DcLinkTrip::OVER);
}

static void InvalidLimitsFailSafe()
{
    const s32fp limits[][2] = {
        { Min, 0 }, { Min, FP_FROMINT(-5) }, { Max, Min }, { Min, Min }
    };
    for (unsigned i = 0; i < sizeof(limits) / sizeof(limits[0]); ++i)
    {
        DcLinkTrip trip = Make();
        trip.Update(FP_FROMINT(50), limits[i][0], limits[i][1], false);
        assert(trip.State() != DcLinkTrip::OK);
    }
}

static void UnconfiguredDebounceFailsSafe()
{
    DcLinkTrip trip;
    trip.Update(FP_FROMINT(53), Min, Max, true);
    assert(trip.State() != DcLinkTrip::OK);
}

int main()
{
    InRangeDoesNotTrip();
    OverVoltageTripsOnFirstSample();
    OverVoltageTripsEvenWhenNotRunning();
    LimitsThemselvesAreAllowed();
    UnderVoltageNeedsConsecutiveSamples();
    UnderVoltageGlitchIsForgiven();
    UnderVoltageIgnoredWhileNotRunning();
    UnderVoltageCountRestartsAfterOutputWasOff();
    TripIsLatchedUntilReset();
    FirstFaultIsKept();
    InvalidLimitsFailSafe();
    UnconfiguredDebounceFailsSafe();
    puts("DC link trip checks passed");
}
