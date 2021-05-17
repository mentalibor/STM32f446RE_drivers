/******************************************************************************
** Copyright (c) 2021 Mentalibor. All rights reserved. Email: mentalibor@gmail.com
**
** Name
** stm32f44xx_gpio.c
**
** Purpose
** GPIO driver header file for the STM32F44XX
**
** Revision
** 08-Feb-2021 (Mentalibor) Initial version.
**
******************************************************************************/

#ifndef INC_STM32F44XX_GPIO_H_
#define INC_STM32F44XX_GPIO_H_

#include "stm32f44xx.h"

/*configuration structure of the GPIO pin*/
typedef struct {
	uint8_t GPIO_PinNumber;					/*possible values from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;					/*possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;					/*possible values from @GPIO_PIN_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;			/*possible values from @GPIO_PULL_UP_DOWN_MODES*/
	uint8_t GPIO_PinOPType;					/*possible values from @GPIO_OUTPUT_TYPES*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*handle structure of the GPIO pin*/
typedef struct {
	GPIO_RegDef_t* pGPIOx;					/*this holds the base address of the corresponding GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*this holds the GPIO pin configuration settings*/
} GPIO_Handler_t;

/*
 * @GPIO_PIN_NUMBERS
 */

#define GPIO_PIN_NO_0 								0
#define GPIO_PIN_NO_1 								1
#define GPIO_PIN_NO_2 								2
#define GPIO_PIN_NO_3 								3
#define GPIO_PIN_NO_4								4
#define GPIO_PIN_NO_5 								5
#define GPIO_PIN_NO_6 								6
#define GPIO_PIN_NO_7 								7
#define GPIO_PIN_NO_8 								8
#define GPIO_PIN_NO_9 								9
#define GPIO_PIN_NO_10 								10
#define GPIO_PIN_NO_11								11
#define GPIO_PIN_NO_12 								12
#define GPIO_PIN_NO_13 								13
#define GPIO_PIN_NO_14 								14
#define GPIO_PIN_NO_15 								15


/*
 * @GPIO_PIN_MODES
 */
#define GPIO_MODE_INPUT 							0
#define GPIO_MODE_OUTPUT							1
#define GPIO_MODE_ALTERNAT 							2
#define GPIO_MODE_ANALOG		 					3
#define GPIO_MODE_INT_FALL_EDGE 					4
#define GPIO_MODE_INT_RISE_EDGE						5
#define GPIO_MODE_INT_RISE_FALL_EDGE 				6

/*
 * @GPIO_OUTPUT_TYPES
 */
#define GPIO_OUTPUT_TYPE_PULL_UP 					0
#define GPIO_OUTPUT_TYPE_OPEN_DRAIN 				1

/*
 * @GPIO_PIN_SPEEDS
 */
#define GPIO_OUTPUT_SPEED_LOW 						0
#define GPIO_OUTPUT_SPEED_MEDIUM 					1
#define GPIO_OUTPUT_SPEED_FAST 						2
#define GPIO_OUTPUT_SPEED_HIGH 						3

/*
 * GPIO_PULL_UP_DOWN_MODES
 */
#define GPIO_PIN_NO_PULL_UP_DOWN					0
#define GPIO_PIN_PULL_UP 							1
#define GPIO_PIN_PULL_DOWN 							2

/*********************************************************************************************************************************
 * 												APIs supported by this driver
 *********************************************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]   pGPIOx  Reference to the GPIO definition structure
 * @param[in]   EnorDi  Enable or disable macro
 *
 ******************************************************************************/
void GPIO_PerClockControl	(GPIO_RegDef_t *pGPIOx
							, uint8_t EnorDi
							);

/*
 * Initialization and deinitialization
 */
/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Initializes the given GPIO peripheral
 *
 * @param[in]   pGPIOHandler  Reference to the GPIO handler
 *
 ******************************************************************************/
void GPIO_Init	(GPIO_Handler_t *pGPIOHandler);

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Denitializes the given GPIO peripheral
 *
 * @param[in]   pGPIOHandler  Reference to the GPIO handler
 *
 ******************************************************************************/
void GPIO_DeInit	(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Reads from the given GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]	PinNumber 	Number of GPIO pin
 *
 * @return uint8_t
 * @retval 0 - low state
 * @retval 1 - high state
 ******************************************************************************/
uint8_t GPIO_ReadFromInputPin	(GPIO_RegDef_t *pGPIOx
								,uint8_t PinNumber
								);

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Reads from the given GPIO port
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 *
 * @return uint16_t
 * @retval 0x0 - 0xFFFF - port pin [0,1,x.,15] states
 ******************************************************************************/
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx);

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Writes a value to the GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]   PinNumber	GPIO pin number
 * @param[in]	Value		Set/Reset values
 *  ******************************************************************************/
void GPIO_WriteToOutputPin	(GPIO_RegDef_t *pGPIOx
							,uint8_t PinNumber
							,uint8_t Value
							);

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Writes a value to the GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]	Value		0x0 - 0xFFFF
 *  ******************************************************************************/
void GPIO_WriteToOutputPort	(GPIO_RegDef_t *pGPIOx
							,uint16_t Value
							);

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Toggles the state of the given GPIO pin
 *
 * @param[in]   pGPIOx  	Reference to the GPIO definition structure
 * @param[in]	PinNumber 	Number of GPIO pin
 *  ******************************************************************************/
void GPIO_ToggleOutputPin	(GPIO_RegDef_t *pGPIOx
							,uint8_t PinNumber
							);

/*
 * IRQ configuration and ISR handling
 */
/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Enables or disables the interrupt registers for the corresponding GPIO
 * 		  port
 *
 * @param[in]   IRQ_Number  	IRQ number
 * @param[in]	EnorDi		 	enable or disable macro
 *  ******************************************************************************/
void GPIO_IRQInterruptConfig	(uint8_t IRQ_Number
								,uint8_t EnorDi
								);

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Configures the interrupt priority register
 *
 * @param[in]   IRQ_Number  	IRQ number
 * @param[in]	IRQ_Priority	IRQ priority value
 *  ******************************************************************************/
void GPIO_IRQPriorityConfig	(uint8_t IRQ_Number
							,uint32_t IRQ_Priority
							);

/** *****************************************************************************
 * @ingroup api_gpio
 * @brief Clears the PR register of the EXTI after invoking interrupt
 *
 * @param[in]   PinNumber  		GPIO pin number
 *  ******************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F44XX_GPIO_H_ */
