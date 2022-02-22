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

#ifndef CURRENTVOLTAGEDRIVER_H
#define CURRENTVOLTAGEDRIVER_H

#include <stdint.h>

namespace c2000 {

class CurrentVoltageDriver {
public:
    static void Init();

    static uint16_t Phase1()
    {
        getMeasurement();
        return measureACurrent;
    }

    static uint16_t Phase2() {
        return measureBCurrent;
    }

    static uint16_t getHVDC() {
      return measureDCV;
    }
    
    static volatile int32_t measureACurrent;
    static volatile int32_t measureBCurrent;
    static volatile int32_t measureDCV;

    static volatile uint32_t cycles;

    private:
        static void getMeasurement();
};

}  // namespace c2000

#endif // CURRENTVOLTAGEDRIVER_H
