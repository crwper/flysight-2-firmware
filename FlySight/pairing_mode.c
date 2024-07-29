/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2024 Bionic Avionics Inc.                                   **
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

#include "main.h"
#include "app_ble.h"
#include "app_common.h"
#include "led.h"
#include "mode.h"

static void FS_PairingMode_Callback(void)
{
	FS_Mode_PushQueue(FS_MODE_EVENT_FORCE_UPDATE);
}

void FS_PairingMode_Init(void)
{
	// Initialize LEDs
	FS_LED_SetColour(FS_LED_GREEN);
	FS_LED_Pulse();

	// Request pairing
	APP_BLE_RequestPairing(FS_PairingMode_Callback);
}

void FS_PairingMode_DeInit(void)
{
	// Turn off LEDs
	FS_LED_Off();

	// Cancel pairing
	APP_BLE_CancelPairing();
}
