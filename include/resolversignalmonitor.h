// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef RESOLVERSIGNALMONITOR_H
#define RESOLVERSIGNALMONITOR_H

#include <stdint.h>

namespace encoder {

/** Live resolver signal-quality check.
 * Uses the instantaneous radius sin^2 + cos^2, which for a healthy resolver is
 * independent of the rotor angle, so it works with the rotor parked and never
 * freezes. Hysteresis between the lock and unlock levels plus a consecutive
 * sample debounce stop it chattering.
 */
class ResolverSignalMonitor
{
public:
    //! Radius (counts) at or above which the signal counts as good
    static const int32_t LockRadius = 600;
    //! Radius (counts) below which the signal counts as lost
    static const int32_t UnlockRadius = 350;
    //! Consecutive samples needed to change state
    static const uint16_t DebounceSamples = 16;

    ResolverSignalMonitor() : locked(false), usable(false), count(0) {}

    void Reset()
    {
        locked = false;
        usable = false;
        count = 0;
    }

    void Update(int16_t sin, int16_t cos)
    {
        // Each square is at most 2^30, so the sum fits an unsigned 32-bit value.
        const uint32_t radiusSq = (uint32_t)((int32_t)sin * sin) + (uint32_t)((int32_t)cos * cos);
        const bool     change = locked ? radiusSq < (uint32_t)(UnlockRadius * UnlockRadius)
                                       : radiusSq >= (uint32_t)(LockRadius * LockRadius);
        if (!change)
        {
            count = 0;
        }
        else if (++count >= DebounceSamples)
        {
            locked = !locked;
            count = 0;
        }
        usable = locked && radiusSq >= (uint32_t)(UnlockRadius * UnlockRadius);
    }

    bool Locked() const { return locked; }

    //! True if the latest sample may be decoded: locked, and that sample itself
    //! above the unlock level. Stays false for bad samples during the debounce
    //! window before an unlock is declared.
    bool SampleUsable() const { return usable; }

private:
    bool locked;
    bool usable;
    uint16_t count;
};

} // namespace encoder

#endif
