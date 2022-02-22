/*
 * This file is part of the stm32-sine project.
 *
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
#include "c2000/encoder.h"
#include "c2000/encoderdriver.h"

#include "errormessage.h"
#include "my_math.h"
#include "params.h"
#include "sine_core.h"

#include <device.h>

namespace c2000 {

const uint16_t MIN_RES_AMP = 1000;
#define TWO_PI 65536

static int32_t           resolverMin = 0, resolverMax = 0, startupDelay;
static volatile uint16_t angle = 0;
static int32_t           turnsSinceLastSample = 0;
static uint32_t          fullTurns = 0;

volatile int32_t Encoder::measuredSin = 0, Encoder::measuredCos = 0, Encoder::exciterMonitor = 0;

bool Encoder::SeenNorthSignal()
{
    return true;
}

void Encoder::UpdateRotorAngle(int dir)
{
    static uint16_t lastAngle = 0;
    static int      poleCounter = 0;

    angle = GetAngleSinCos(dir);
    UpdateTurns(angle, lastAngle);

   if (lastAngle <= (TWO_PI / 2) && angle > (TWO_PI / 2))
   {
      if (poleCounter == 0)
      {
         fullTurns++;
         poleCounter = Param::GetInt(Param::respolepairs);
      }
      else
      {
         poleCounter--;
      }
   }

   startupDelay = startupDelay > 0 ? startupDelay - 1 : 0;
   lastAngle = angle;
}

int32_t Encoder::GetAngleSinCos(int dir)
{
    bool invert = false;

    EncoderDriver::getSinCos(measuredSin, measuredCos, exciterMonitor);

    // Wait for signal to reach usable amplitude
    if ((resolverMax - resolverMin) > MIN_RES_AMP)
    {
        if (invert)
            return SineCore::Atan2(-measuredSin, -measuredCos);
        return SineCore::Atan2(measuredSin, measuredCos);
    }
    else
    {
        int temp = MIN(measuredSin, measuredCos);
        resolverMin = MIN(temp, resolverMin);
        temp = MAX(measuredSin, measuredCos);
        resolverMax = MAX(temp, resolverMax);

        if (0 == startupDelay)
        {
            // ErrorMessage::Post(ERR_LORESAMP);
        }
        return 0;
    }
}

void Encoder::UpdateTurns(uint16_t angle, uint16_t lastAngle)
{
    int signedDiff = (int)angle - (int)lastAngle;
    int absDiff = ABS(signedDiff);
    int sign = signedDiff < 0 ? -1 : 1;

    if (absDiff > (TWO_PI / 2)) // wrap detection
    {
        sign = -sign;
        signedDiff += sign * TWO_PI;
        absDiff = ABS(signedDiff);
    }

    turnsSinceLastSample += signedDiff;
}

void Encoder::UpdateRotorFrequency(int callingFrequency)
{
}

void Encoder::SetPwmFrequency(uint32_t frq)
{
}

uint16_t Encoder::GetRotorAngle()
{
    return angle;
}

u32fp Encoder::GetRotorFrequency()
{
    return 0;
}

int Encoder::GetRotorDirection()
{
    return 0;
}

} // namespace c2000
