/*
 * stm32f44xx_rcc.h
 *
 *  Created on: Feb 6, 2021
 *      Author: Mentalibor
 */

#ifndef INC_STM32F44XX_RCC_H_
#define INC_STM32F44XX_RCC_H_

#include "stm32f44xx.h"

/*******************************************************************************
 * @ingroup api_rcc
 * @brief Calculates the APB1 bus clock value
 *
 * @return uint32_t
 * @retval 0x0 - 0xFFFFFFFF clock value
 ******************************************************************************/
uint32_t RCC_GetPCLK1Value(void);

/*******************************************************************************
 * @ingroup api_rcc
 * @brief Calculates the APB2 bus clock value
 *
 * @return uint32_t
 * @retval 0x0 - 0xFFFFFFFF clock value
 ******************************************************************************/
uint32_t RCC_GetPCLK2Value(void);

uint32_t RCC_GetPLLOutputClock();

#endif /* INC_STM32F44XX_RCC_H_ */
