// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef OVERCURRENTTRIP_H
#define OVERCURRENTTRIP_H

#include "my_fp.h"
#include "my_math.h"

/** Latched software over-current trip on the phase currents.
 * Only two phases are measured; the third follows from il1 + il2 + il3 = 0.
 */
class OverCurrentTrip
{
public:
    OverCurrentTrip() : tripped(false) {}

    void Update(s32fp il1, s32fp il2, s32fp limit)
    {
        const s32fp il3 = -(il1 + il2);
        if (limit <= 0 || ABS(il1) > limit || ABS(il2) > limit || ABS(il3) > limit)
            tripped = true;
    }

    void Reset() { tripped = false; }

    bool Tripped() const { return tripped; }

private:
    volatile bool tripped;
};

#endif
