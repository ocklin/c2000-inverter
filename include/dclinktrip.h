// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef DCLINKTRIP_H
#define DCLINKTRIP_H

#include "my_fp.h"
#include <stdint.h>

/** Latched DC-link over- and under-voltage trip.
 * Over-voltage trips on the first sample, regardless of output state.
 * Under-voltage only counts while the output is running and needs
 * undervoltSamples consecutive samples. The first fault is kept.
 */
class DcLinkTrip
{
public:
    enum Fault { OK, OVER, UNDER };

    DcLinkTrip() : undervoltSamples(0), underCount(0), fault(OK) {}

    void Configure(uint16_t samples)
    {
        undervoltSamples = samples;
        underCount = 0;
        // Reconfiguring must never clear a latched fault.
    }

    void Update(s32fp udc, s32fp minimum, s32fp maximum, bool running)
    {
        if (fault != OK)
            return;

        if (undervoltSamples == 0 || maximum <= 0 || minimum >= maximum)
        {
            fault = OVER;
        }
        else if (udc > maximum)
        {
            fault = OVER;
        }
        else if (running && udc < minimum)
        {
            underCount = underCount + 1U;
            if (underCount >= undervoltSamples)
                fault = UNDER;
        }
        else
        {
            underCount = 0;
        }
    }

    void Reset()
    {
        fault = OK;
        underCount = 0;
    }

    Fault State() const { return fault; }

private:
    volatile uint16_t undervoltSamples;
    volatile uint16_t underCount;
    volatile Fault fault;
};

#endif
