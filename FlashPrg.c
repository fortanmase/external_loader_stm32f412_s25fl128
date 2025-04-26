/**************************************************************************/ /**
                                                                              * @file     FlashPrg.c
                                                                              * @brief    Flash Programming Functions adapted for New Device Flash
                                                                              * @version  V1.0.0
                                                                              * @date     10. January 2018
                                                                              ******************************************************************************/
/*
 * Copyright (c) 2010-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "FlashOS.h" // FlashOS Structures
#include "main.h"
#include "s25fl128s.h"

#include <stdio.h>
#include <string.h>

/*
   Mandatory Flash Programming Functions (Called by FlashOS):
                int Init        (unsigned long adr,   // Initialize Flash
                                 unsigned long clk,
                                 unsigned long fnc);
                int UnInit      (unsigned long fnc);  // De-initialize Flash
                int EraseSector (unsigned long adr);  // Erase Sector Function
                int ProgramPage (unsigned long adr,   // Program Page Function
                                 unsigned long sz,
                                 unsigned char *buf);

   Optional  Flash Programming Functions (Called by FlashOS):
                int BlankCheck  (unsigned long adr,   // Blank Check
                                 unsigned long sz,
                                 unsigned char pat);
                int EraseChip   (void);               // Erase complete Device
      unsigned long Verify      (unsigned long adr,   // Verify Function
                                 unsigned long sz,
                                 unsigned char *buf);

       - BlanckCheck  is necessary if Flash space is not mapped into CPU memory space
       - Verify       is necessary if Flash space is not mapped into CPU memory space
       - if EraseChip is not provided than EraseSector for all sectors is called
*/

extern QSPI_HandleTypeDef hqspi;
extern UART_HandleTypeDef huart3;
static uint8_t buffer[128];
static unsigned int DevAddr;

S25FL128S_Interface_t mode = S25FL128S_QPI_MODE;

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{
  mcu_init();

  /* ----- Debug ----- */
  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

  // sprintf((char *)buffer, "\r\n---------- %s ----------\r\n", __func__);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  // sprintf((char *)buffer, "adr: 0x%lx, clk: %ld Hz, fnc: %ld\r\n", adr, clk, fnc);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  // /* ----- QSPI Init code ----- */
  int ret = 0;
  volatile int i;
  volatile unsigned char *ptr = (volatile unsigned char *)&hqspi;

  DevAddr = adr;

  uint8_t id = 0U;
  ret = S25FL128S_ReadID(&hqspi, S25FL128S_SPI_MODE, &id);
  ret |= S25FL128S_ResetEnable(&hqspi, mode);
  ret |= S25FL128S_ResetMemory(&hqspi, mode);
  ret |= S25FL128S_AutoPollingMemReady(&hqspi, mode);
  ret |= S25FL128S_Enter4BytesAddressMode(&hqspi, mode);

  // snprintf((char *)buffer, sizeof(buffer), "Flash ID: 0x%02X | RetVal: 0x%02X\r\n", id, ret);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  return ret;
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit(unsigned long fnc)
{
  int ret = 0;

  // sprintf((char *)buffer, "\r\n---------- %s ----------\r\n", __func__);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  // sprintf((char *)buffer, "fnc: %ld\r\n", fnc);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  if (hqspi.State != HAL_QSPI_STATE_BUSY_MEM_MAPPED)
  {
    ret = S25FL128S_EnableMemoryMappedModeSTR(&hqspi, mode);
    // sprintf((char *)buffer, "Memory Mapped Mode Enable RetVal: %d\r\n", ret);
    // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);
  }

  return ret;
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip(void)
{
  int ret = 1;
  // sprintf((char *)buffer, "\r\n---------- %s ----------\r\n", __func__);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  ret = S25FL128S_WriteEnable(&hqspi, mode);
  ret |= S25FL128S_ChipErase(&hqspi, mode);
  ret |= S25FL128S_AutoPollingMemReady(&hqspi, mode);

  // snprintf((char *)buffer, sizeof(buffer), "Erase Chip RetVal: %d\r\n", ret);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  /* Add your Code */
  return ret; // Finished without Errors
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector(unsigned long adr)
{
  int ret = 1;

  // sprintf((char *)buffer, "\r\n---------- %s ----------\r\n", __func__);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  // sprintf((char *)buffer, "mcu adr: 0x%lx\r\n", adr);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  if (adr >= DevAddr)
  {
    adr -= DevAddr;

    // sprintf((char *)buffer, "flash adr: 0x%lx\r\n", adr);
    // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

    ret = S25FL128S_WriteEnable(&hqspi, mode);
    ret |= S25FL128S_BlockErase(&hqspi, mode, adr, S25FL128S_ERASE_4K);
    ret |= S25FL128S_AutoPollingMemReady(&hqspi, mode);
  }

  // snprintf((char *)buffer, sizeof(buffer), "Erase Sector RetVal: %d\r\n", ret);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  return (ret);
}

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  // sprintf((char *)buffer, "\r\n---------- %s ----------\r\n", __func__);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  // sprintf((char *)buffer, "adr: 0x%lx, sz: %ld\r\n", adr, sz);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  int ret = 1;

  if (adr >= DevAddr)
  {
    adr -= DevAddr;
    ret = S25FL128S_WritePages(&hqspi, mode, (uint8_t *)buf, adr, sz);
  }

  /* Add your Code */
  return ret; // Finished without Errors
}

unsigned long Verify(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  // sprintf((char *)buffer, "\r\n---------- %s ----------\r\n", __func__);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  // sprintf((char *)buffer, "adr: 0x%lx, sz: %ld\r\n", adr, sz);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  volatile unsigned long ret = adr + sz;
  volatile int i;
  int flash_ret = 1;

  if (hqspi.State != HAL_QSPI_STATE_BUSY_MEM_MAPPED)
  {
    flash_ret = S25FL128S_EnableMemoryMappedModeSTR(&hqspi, mode);
    if (flash_ret != 0)
    {
      ret = 0U;
    }
  }

  if (ret != 0U)
  {
    for (i = 0; i < sz; i++)
    {
      if ((*((volatile unsigned char *)(adr + i))) != buf[i])
      {
        // Verification Failed (return address)
        ret = adr + i;
        break;
      }
    }
  }

  return (ret);
}

int BlankCheck(unsigned long adr, unsigned long sz, unsigned char pat)
{
  int ret = 0;
  volatile int i;

  // sprintf((char *)buffer, "\r\n---------- %s ----------\r\n", __func__);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  // sprintf((char *)buffer, "adr: 0x%lx, sz: %ld, pat: 0x%02x\r\n", adr, sz, pat);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  if (hqspi.State != HAL_QSPI_STATE_BUSY_MEM_MAPPED)
  {
    ret = S25FL128S_EnableMemoryMappedModeSTR(&hqspi, mode);
    // sprintf((char *)buffer, "Memory Mapped Mode Enable RetVal: %d\r\n", ret);
    // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);
  }

  if (ret == 0)
  {
    // snprintf((char *)buffer, sizeof(buffer), "Performing Blank Check\r\n");
    // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

    for (i = 0; i < sz; i++)
    {
      if ((*((volatile unsigned char *)(adr + i))) != pat)
      {
        ret = 1;
        break;
      }
    }
  }

  // sprintf((char *)buffer, "Blank Check RetVal: %d\r\n", ret);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  uint8_t id = 0U;
  int flash_ret = 0;
  QUADSPI_Init();
  flash_ret = S25FL128S_ReadID(&hqspi, S25FL128S_SPI_MODE, &id);
  flash_ret |= S25FL128S_ResetEnable(&hqspi, mode);
  flash_ret |= S25FL128S_ResetMemory(&hqspi, mode);
  flash_ret |= S25FL128S_AutoPollingMemReady(&hqspi, mode);
  flash_ret |= S25FL128S_Enter4BytesAddressMode(&hqspi, mode);

  if (flash_ret != 0)
  {
    HAL_Delay(100U);

    QUADSPI_Init();
    flash_ret = S25FL128S_ReadID(&hqspi, S25FL128S_SPI_MODE, &id);
    flash_ret |= S25FL128S_ResetEnable(&hqspi, mode);
    flash_ret |= S25FL128S_ResetMemory(&hqspi, mode);
    flash_ret |= S25FL128S_AutoPollingMemReady(&hqspi, mode);
    flash_ret |= S25FL128S_Enter4BytesAddressMode(&hqspi, mode);
  }

  // snprintf((char *)buffer, sizeof(buffer), "Flash ID: 0x%02X | RetVal: 0x%02X\r\n", id, ret);
  // HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

  /* Add your Code */
  return ret;
}