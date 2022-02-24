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

#ifndef BREAKSWITCHDRIVER_H
#define _BREAKSWITCHDRIVER_H

#include <stdint.h>

namespace c2000 {

class BreakSwitchDriver {
public: 

    static void Init();
    static void ReadValues(uint16_t & no, uint16_t & nc);
};

} // namespace c2000

#endif // _BREAKSWITCHDRIVER_H
