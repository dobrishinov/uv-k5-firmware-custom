/* Original work Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Modified work Copyright 2024 kamilsss655
 * https://github.com/kamilsss655
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>

void     BOARD_FLASH_Init(void);
void     BOARD_GPIO_Init(void);
void     BOARD_PORTCON_Init(void);
void     BOARD_ADC_Init(void);
void     BOARD_ADC_GetBatteryInfo(uint16_t *pVoltage);
void     BOARD_Init(void);
void     BOARD_EEPROM_Init(void);
void     BOARD_EEPROM_LoadCalibration(void);
uint32_t BOARD_fetchChannelFrequency(const int channel);
void     BOARD_FactoryReset(bool bIsAll);
#ifdef ENABLE_SPECTRUM_SHOW_CHANNEL_NAME
void     BOARD_gMR_LoadChannels();
int      BOARD_gMR_fetchChannel(const uint32_t freq);
#endif
#ifdef ENABLE_MESSENGER
void BOARD_GetDeviceUniqueId(char *buffer);
#endif
#ifdef ENABLE_TEMP_SENSOR
int8_t BOARD_GetDeviceTemperature();
#endif

#endif

