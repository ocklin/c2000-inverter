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

#include "c2000/breakswitchdriver.h"

#include <device.h>

// pin 7, GPIO15	BRAKE N/O	BRAKE NORMALLY OPEN. 3.8V OPEN , 0.27V CLOSED
// pin 27, GPIO26	BRAKE N/C	BRAKE NORMALLY CLOSED. 3.8V OPEN , 0.27V CLOSED

namespace c2000 {

void BreakSwitchDriver::Init() {
    //
    // Make GPIO15 an input on GPIO15
    //
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO15
    GPIO_setPinConfig(GPIO_15_GPIO15);               // GPIO15 = GPIO15
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_IN);     // GPIO15 = input

    //
    // Make GPIO17 an input on GPIO27
    //
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO27
    GPIO_setPinConfig(GPIO_27_GPIO27);               // GPIO27 = GPIO27
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_IN);     // GPIO27 = input
}

void BreakSwitchDriver::ReadValues(uint16_t & no, uint16_t & nc) {
    no = GPIO_readPin(15);
    nc = GPIO_readPin(27);
}

} // namespace c2000
