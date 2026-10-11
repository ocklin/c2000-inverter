// SPDX-License-Identifier: GPL-3.0-or-later
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "overcurrenttrip.h"
#include <assert.h>
#include <stdio.h>

static const s32fp Limit = FP_FROMINT(20);

static void NoTripBelowLimit()
{
    OverCurrentTrip trip;
    trip.Update(FP_FROMINT(10), FP_FROMINT(-10), Limit);
    assert(!trip.Tripped());
    trip.Update(0, 0, Limit);
    assert(!trip.Tripped());
}

static void TripsOnEitherMeasuredPhaseInEitherDirection()
{
    const s32fp over = FP_FROMINT(21);
    const s32fp cases[][2] = { { over, 0 }, { -over, 0 }, { 0, over }, { 0, -over } };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        OverCurrentTrip trip;
        trip.Update(cases[i][0], cases[i][1], Limit);
        assert(trip.Tripped());
    }
}

static void LimitItselfIsAllowed()
{
    OverCurrentTrip trip;
    trip.Update(Limit, 0, Limit);
    trip.Update(0, -Limit, Limit);
    assert(!trip.Tripped());
}

static void ThirdPhaseIsChecked()
{
    // Two phases at 14 A in the same direction: the unmeasured third is -28 A.
    OverCurrentTrip trip;
    trip.Update(FP_FROMINT(14), FP_FROMINT(14), Limit);
    assert(trip.Tripped());
}

static void BalancedLoadWithinLimitDoesNotTrip()
{
    // 17.32 A peak balanced: il1 = 17.32, il2 = -8.66, il3 = -8.66.
    OverCurrentTrip trip;
    trip.Update(FP_FROMFLT(17.32), FP_FROMFLT(-8.66), Limit);
    assert(!trip.Tripped());
}

static void TripIsLatchedUntilReset()
{
    OverCurrentTrip trip;
    trip.Update(FP_FROMINT(25), 0, Limit);
    assert(trip.Tripped());
    trip.Update(0, 0, Limit);
    assert(trip.Tripped());
    trip.Reset();
    assert(!trip.Tripped());
    trip.Update(0, 0, Limit);
    assert(!trip.Tripped());
}

static void InvalidLimitFailsSafe()
{
    const s32fp limits[] = { 0, FP_FROMINT(-20) };
    for (unsigned i = 0; i < sizeof(limits) / sizeof(limits[0]); ++i)
    {
        OverCurrentTrip trip;
        trip.Update(0, 0, limits[i]);
        assert(trip.Tripped());
    }
}

int main()
{
    NoTripBelowLimit();
    TripsOnEitherMeasuredPhaseInEitherDirection();
    LimitItselfIsAllowed();
    ThirdPhaseIsChecked();
    BalancedLoadWithinLimitDoesNotTrip();
    TripIsLatchedUntilReset();
    InvalidLimitFailsSafe();
    puts("Over-current trip checks passed");
}
