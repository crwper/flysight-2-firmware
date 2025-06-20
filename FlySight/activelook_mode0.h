/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#ifndef ACTIVELOOK_MODE0_H
#define ACTIVELOOK_MODE0_H

#include "activelook.h"

/**
 * Called once when the mode is selected, to reset and load config data.
 */
void FS_ActiveLook_Mode0_Init(void);

/**
 * Called repeatedly to perform multi-step layout setup. Each call
 * sends exactly one BLE command. Returns FS_AL_SETUP_DONE when complete.
 */
FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void);

/**
 * Called whenever we want to redraw or update the data on the display.
 * Uses the config-based line definitions from FS_ActiveLook_Mode0_Init().
 */
void FS_ActiveLook_Mode0_Update(void);

#endif /* ACTIVELOOK_MODE0_H */
