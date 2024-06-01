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
#include "app_common.h"
#include "app_fatfs.h"
#include "config.h"
#include "gnss.h"
#include "resource_manager.h"
#include "start_control.h"
#include "state.h"

extern UART_HandleTypeDef huart1;

void FS_StartMode_Init(void)
{
	/* Initialize FatFS */
	FS_ResourceManager_RequestResource(FS_RESOURCE_FATFS);

	/* Initialize controller */
	FS_StartControl_Init();

	/* Initialize configuration */
	FS_Config_Init();
	if (FS_Config_Read("/config.txt") != FS_CONFIG_OK)
	{
		FS_Config_Write("/config.txt");
	}

	/* Read selectable config */
	if (f_chdir("/config") == FR_OK)
	{
		FS_Config_Read(FS_State_Get()->config_filename);
	}

	/* Enable USART */
	MX_USART1_UART_Init();

	/* Initialize GNSS */
	FS_GNSS_Init();

	/* Start GNSS */
	FS_GNSS_Start();
}

void FS_StartMode_DeInit(void)
{
	/* Disable controller */
	FS_StartControl_DeInit();

	/* Disable GNSS */
	FS_GNSS_DeInit();

	/* Disable USART */
	HAL_UART_DeInit(&huart1);

	/* De-initialize FatFS */
	FS_ResourceManager_ReleaseResource(FS_RESOURCE_FATFS);
}
