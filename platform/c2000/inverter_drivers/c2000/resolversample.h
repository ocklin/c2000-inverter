// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef C2000RESOLVERSAMPLE_H
#define C2000RESOLVERSAMPLE_H

#include "c2000/motoranalogcapture.h"
#include "params.h"
#include "resolvercalibration.h"
#include <stdint.h>

namespace c2000 {

/**
 * Resolver sample source for encoder::ResolverEncoder: the raw ADC readings
 * centred on their calibrated offsets, with the cosine scaled to the sine's
 * amplitude (resofssin, resofscos, resgaincos; set by the rescal calibration).
 */
struct CalibratedResolverSample
{
    static int16_t ResolverSine()
    {
        return encoder::CorrectResolverSample(
            MotorAnalogCapture::ResolverSine(), Param::GetInt(Param::resofssin), 100);
    }

    static int16_t ResolverCosine()
    {
        return encoder::CorrectResolverSample(
            MotorAnalogCapture::ResolverCosine(), Param::GetInt(Param::resofscos),
            Param::GetInt(Param::resgaincos));
    }
};

} // namespace c2000

#endif // C2000RESOLVERSAMPLE_H
