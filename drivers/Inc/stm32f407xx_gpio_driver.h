/*
 * stm32f407xx_gpio_driver.h
 *
 * THIS HEADER FILE CONTAIN DRIVER SPECIFIC DATA AND APIs
 *
 *  Created on: Aug 14, 2024
 *      Author: sachithra
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"



/*
 * This is the Configuration Structure for GPIO pin
 */

typedef struct
{
	// We have used uint8_t instead of uint32_t because 1 byte is sufficient here, since pin numbers from 0 - 15
	uint8_t GPIO_PinNumber;						/*|< Possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;						/*|< Possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunction;

}GPIO_PinConfig_t;





/*
 * This is the Handle Structure for GPIO pin
 */

typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;						/*|< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;			/*|< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible pin numbers
 */

#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15



/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3

#define GPIO_MODE_IT_FT			4				/*|< Interrupt Falling Edge Trigger >*/
#define GPIO_MODE_IT_RT			5				/*|< Interrupt Rising Edge Trigger >*/
#define GPIO_MODE_IT_RFT		6				/*|< Interrupt Rising Edge Falling Edge Trigger >*/


/*
 * GPIO pin possible output types
 */

#define	GPIO_OP_TYPE_PP			0				/*|< Output Push-Pull (Reset state) >*/
#define	GPIO_OP_TYPE_OD			1				/*|< Output Open Drain >*/



/*
 * GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3



/*
 * GPIO pin possible pull up and pull down register
 */

#define GPIO__NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2



/******************************************************************************************
 * 								APIs supported by this driver
 * 					For more information about the APIs check the function definitions
 ******************************************************************************************
 */



/*
 * Peripheral Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);



/*
 * Init and DeInit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);



/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);







#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
