/*
 * This file is part of the Tesla M3 OSS Inverter project.
 *
 * Copyright (C) 2022 Bernd Ocklin <bernd@ocklin.de>
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

#include <stdint.h>

#ifndef ENCODERDRIVER_H
#define ENCODERDRIVER_H

namespace c2000 {

/*
  Class taking care of C2000 specific API's for the position encoder:
  
     PWM for the exciter and ADC for measuring sin/cos
*/
class EncoderDriver {

public:
    static void Init(uint16_t pwmmax);
    static void getSinCos(volatile int32_t &sin, volatile int32_t &cos, volatile int32_t &monitor);

    static volatile uint32_t cycles;

private:
    static void InitAdc();
};

} // namespace c2000

#endif //ENCODERDRIVER_H
