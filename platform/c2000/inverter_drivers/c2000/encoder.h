/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef C2000ENCODER_H
#define C2000ENCODER_H

#include "c2000/resolversample.h"
#include "my_fp.h"
#include "resolverencoder.h"
#include <stdint.h>

namespace c2000 {

/**
 * Resolver-based encoder for the Tesla M3 inverter C2000 platform: a thin
 * wrapper around the host-tested encoder::ResolverEncoder, fed with the
 * calibrated ADC samples.
 */
class Encoder
{
public:
    static void     Reset() { Decoder::Reset(); }
    static bool     SeenNorthSignal() { return Decoder::SeenNorthSignal(); }
    static void     UpdateRotorAngle(int32_t dir) { Decoder::UpdateRotorAngle(dir); }
    static void     UpdateRotorFrequency(int32_t callingFrequency)
    {
        Decoder::UpdateRotorFrequency(callingFrequency);
    }
    static void     SetPwmFrequency(uint32_t frq) { Decoder::SetPwmFrequency(frq); }

    static uint16_t GetRotorAngle() { return Decoder::GetRotorAngle(); }
    static u32fp    GetRotorFrequency() { return Decoder::GetRotorFrequency(); }
    static int32_t  GetRotorDirection() { return Decoder::GetRotorDirection(); }
    static int32_t  GetSpeed() { return Decoder::GetSpeed(); }

private:
    typedef encoder::ResolverEncoder<CalibratedResolverSample> Decoder;
};

} // namespace c2000

#endif // C2000ENCODER_H
