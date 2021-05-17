/*
 * stm32f44xx_i2c.h
 *
 *  Created on: Feb 6, 2021
 *      Author: Mentalibor
 */

#ifndef INC_STM32F44XX_I2C_H_
#define INC_STM32F44XX_I2C_H_

#include "stm32f44xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;			/*possible values from @I2C_SCLSpeed*/
	uint8_t  I2C_DeviceAddress;		/*I2C device address*/
	uint8_t  I2C_AckControl;		/*possible values from @I2C_AckControl*/
	uint8_t  I2C_FMDutyCycle;		/*possible values from @I2C_FMDutyCycle*/

}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* to store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* to store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* to store Tx len > */
	uint32_t 		RxLen;		/* to store Tx len > */
	uint8_t 		TxRxState;	/* to store Communication state > */
	uint8_t 		DevAddr;	/* to store slave/device address > */
    uint32_t        RxSize;		/* to store Rx size  > */
    uint8_t         Sr;			/* to store repeated start value  > */
}I2C_Handle_t;


/*
 * I2C application states
 */
#define I2C_READY 						0
#define I2C_BUSY_IN_RX 					1
#define I2C_BUSY_IN_TX 					2

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 				100000
#define I2C_SCL_SPEED_FM2K  			200000
#define I2C_SCL_SPEED_FM4K 				400000


/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        			1
#define I2C_ACK_DISABLE       			0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2       			0
#define I2C_FM_DUTY_16_9     			1


/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   					( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   				( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB						( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  					( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   					( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 					( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 					( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 					( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 					( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  					( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 					( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 				( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  				RESET
#define I2C_ENABLE_SR   				SET


/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 			0
#define I2C_EV_RX_CMPLT  	 			1
#define I2C_EV_STOP       				2
#define I2C_ERROR_BERR 	 				3
#define I2C_ERROR_ARLO  				4
#define I2C_ERROR_AF    				5
#define I2C_ERROR_OVR   				6
#define I2C_ERROR_TIMEOUT 				7
#define I2C_EV_DATA_REQ         		8
#define I2C_EV_DATA_RCV        			9

/******************************************************************************************
 *								APIs supported by this driver
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Enables or disables the I2C peripheral clock
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void I2C_PeriClockControl	(I2C_RegDef_t *pI2Cx
							, uint8_t EnorDi
							);

/*
 * Init and De-init
 */

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Initializes the I2C peripheral
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Deinitializes the I2C peripheral
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 *
 ******************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive
 */

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Sends data as master in blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the TX buffer
 * @param[in]	Len			Length of the TX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 ******************************************************************************/
void I2C_MasterSendData	( I2C_Handle_t *pI2CHandle
						, uint8_t *pTxbuffer
						, uint32_t Len
						, uint8_t SlaveAddr
						, uint8_t Sr
						);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Receives data as master in blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the RX buffer
 * @param[in]	Len			Length of the RX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 ******************************************************************************/
void I2C_MasterReceiveData	( I2C_Handle_t *pI2CHandle
							, uint8_t *pRxBuffer
							, uint8_t Len
							, uint8_t SlaveAddr
							, uint8_t Sr
							);
/*******************************************************************************
 * @ingroup api_i2c
 * @brief Sends data as master in non-blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the TX buffer
 * @param[in]	Len			Length of the TX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 * @return uint8_t
 * @retval I2C_BUSY_IN_TX	Currently sending data
 * @retval I2C_BUSY_IN_RX	Currently receiving data
 * @retbal I2C_READY		Currently ready for data transfer
 ******************************************************************************/
uint8_t I2C_MasterSendDataIT 	( I2C_Handle_t *pI2CHandle
								, uint8_t *pTxbuffer
								, uint32_t Len
								, uint8_t SlaveAddr
								, uint8_t Sr
								);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Receives data as master in non-blocking mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]	pTxbuffer	Reference to the RX buffer
 * @param[in]	Len			Length of the RX buffer
 * @param[in]	SlaveAddr	Slave address
 * @param[in]	Sr			Flag for stop condition bit
 *
 * @return uint8_t
 * @retval I2C_BUSY_IN_TX	Currently sending data
 * @retval I2C_BUSY_IN_RX	Currently receiving data
 * @retbal I2C_READY		Currently ready for data transfer
 ******************************************************************************/
uint8_t I2C_MasterReceiveDataIT ( I2C_Handle_t *pI2CHandle
								, uint8_t *pRxBuffer
								, uint8_t Len
								, uint8_t SlaveAddr
								, uint8_t Sr
								);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Closes receiving data process
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_CloseReceiveData( I2C_Handle_t *pI2CHandle );

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Closes sending data process
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_CloseSendData( I2C_Handle_t *pI2CHandle );

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Perfoms sending data process in slave mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]   data  		data
 *
 ******************************************************************************/
void I2C_SlaveSendData	( I2C_RegDef_t *pI2C
						, uint8_t data
						);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Perfoms receiving data process in slave mode
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]   data  		data
 *
 * @return uint8_t
 * @retval data		0x0 - 0xFF Data
 ******************************************************************************/
uint8_t I2C_SlaveReceiveData( I2C_RegDef_t *pI2C );

/*
 * IRQ Configuration and ISR handling
 */

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Enables or disables the corresponding IRQ
 *
 * @param[in]   IRQNumber   IRQ number
 * @param[in]	EnorDi		Enable or disable macro
 *
 ******************************************************************************/
void I2C_IRQInterruptConfig (uint8_t IRQNumber
							, uint8_t EnorDi
							);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Configures the IRQ priority
 *
 * @param[in]   IRQNumber   IRQ number
 * @param[in]	IRQPriority	IRQ priority
 *
 ******************************************************************************/
void I2C_IRQPriorityConfig	( uint8_t IRQNumber
							, uint32_t IRQPriority
							);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Handles the interrupt event
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Handles the interrupt error
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 *
 ******************************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control APIs
 */

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Enables or disables the I2C peripheral
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void I2C_PeripheralControl	(I2C_RegDef_t *pI2Cx
							, uint8_t EnOrDi
							);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Gets flag status from the SR1 register
 *
 * @param[in]   pI2Cx   	Reference to the I2C definition structure
 * @param[in]   FlagName   	flag name
 *
 * @return uint8_t
 * @retval FLAG_SET
 * @retval FLAG_RESET
 ******************************************************************************/
uint8_t I2C_GetFlagStatus	( I2C_RegDef_t *pI2Cx
							, uint32_t FlagName
							);


/*******************************************************************************
 * @ingroup api_i2c
 * @brief Manages ACKing of I2C peripheral
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 * @param[in]	EnorDi	Enable or disable macro
 *
 ******************************************************************************/
void I2C_ManageAcking	( I2C_RegDef_t *pI2Cx
						, uint8_t EnorDi
						);

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Generates stop condition of the I2C communication
 *
 * @param[in]   pI2Cx  Reference to the I2C definition structure
 *
 ******************************************************************************/
void I2C_GenerateStopCondition( I2C_RegDef_t *pI2Cx );

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Enables or disables the callback events
 *
 * @param[in]   pI2Cx   Reference to the I2C definition structure
 * @param[in]   EnorDi  ENABLE/DISABLE macro
 *
 ******************************************************************************/
void I2C_SlaveEnableDisableCallbackEvents	( I2C_RegDef_t *pI2Cx
											, uint8_t EnorDi
											);

/*
 * Application callback
 */

/*******************************************************************************
 * @ingroup api_i2c
 * @brief Application event callback function
 *
 * @param[in]   pI2CHandle  Reference to the I2C handler
 * @param[in]   AppEv  		Application event macro
 *
 ******************************************************************************/
void I2C_ApplicationEventCallback	( I2C_Handle_t *pI2CHandle
									, uint8_t AppEv
									);

#endif /* INC_STM32F44XX_I2C_H_ */
