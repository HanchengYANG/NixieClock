/*
 * I2C_HAL.c
 *
 *  Created on: 2017年4月21日
 *      Author: hanch
 */

#include "I2C_HAL.h"
#include "fsl_i2c.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/*********             Internal macro             *********/

#define		I2C_RELEASE_BUS_COUNT		100U

/*********             Internal types             *********/

/*********            Internal variables          *********/

static volatile bool I2C_Initialized = false;
static i2c_master_handle_t I2C_Handler;
static volatile bool I2C_CompleteFlag = false;
#ifdef I2C_WITH_RTOS
static volatile bool I2C_InUse = false;
#endif
//static uint8_t I2C_Status;
//static status_t I2C_Result = kStatus_Success;
//static uint8_t I2C_TxBuff[I2C_TX_BUFFER_SIZE];
static i2c_master_transfer_t I2C_MasterXfer = {
		.subaddress = 0,
		.subaddressSize = 0,
};
static i2c_master_config_t I2C_Config = {
		.baudRate_Bps = 100000U,
		.enableMaster = true,
		.enableStopHold = false,
		.glitchFilterWidth = 0
};

/*********      Internal function declaration     *********/

static inline void I2C_InitPins(void);

static inline void I2C_CallBack(I2C_Type* base, i2c_master_handle_t* handler,
		status_t status, void* userData);

static void I2C_ReleaseBus(void);

static void I2C_ReleaseBusDelay(void);

/*********     Internal function implementation   *********/

static inline void I2C_InitPins(void) {
	PORT_SetPinMux(PORTB, I2C_RELEASE_SCL_PIN, kPORT_MuxAlt2); /* PORTB2 (pin 55) is configured as I2C0_SCL */
	PORTB->PCR[2] = ((PORTB->PCR[2]
			& (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK
					| PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
	| PORT_PCR_ODE(PCR_ODE_ENABLED) /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
	| PORT_PCR_PS(PCR_PS_UP) /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
	| PORT_PCR_PE(PCR_PE_ENABLED) /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
	);
	PORT_SetPinMux(PORTB, I2C_RELEASE_SDA_PIN, kPORT_MuxAlt2); /* PORTB3 (pin 56) is configured as I2C0_SDA */
	PORTB->PCR[3] = ((PORTB->PCR[3]
			& (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK
					| PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
	| PORT_PCR_ODE(PCR_ODE_ENABLED) /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
	| PORT_PCR_PS(PCR_PS_UP) /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
	| PORT_PCR_PE(PCR_PE_ENABLED) /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
	);
}

static inline void I2C_CallBack(I2C_Type* base, i2c_master_handle_t* handler,
		status_t status, void* userData) {
	if(status == kStatus_Success) {
		I2C_CompleteFlag = true;
	}
}

static void I2C_ReleaseBus(void) {
	uint8_t i = 0;
	gpio_pin_config_t pin_config;
	port_pin_config_t i2c_pin_config = { 0 };

	/* Config pin mux as gpio */
	i2c_pin_config.pullSelect = kPORT_PullUp;
	i2c_pin_config.mux = kPORT_MuxAsGpio;

	pin_config.pinDirection = kGPIO_DigitalOutput;
	pin_config.outputLogic = 1U;
	CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

	GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
	GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

	/* Drive SDA low first to simulate a start */
	GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
	I2C_ReleaseBusDelay();

	/* Send 9 pulses on SCL and keep SDA low */
	for (i = 0; i < 9; i++) {
		GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
		I2C_ReleaseBusDelay();

		GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
		I2C_ReleaseBusDelay();

		GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
		I2C_ReleaseBusDelay();
		I2C_ReleaseBusDelay();
	}

	/* Send stop */
	GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
	I2C_ReleaseBusDelay();

	GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
	I2C_ReleaseBusDelay();

	GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
	I2C_ReleaseBusDelay();

	GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
	I2C_ReleaseBusDelay();
}

static void I2C_ReleaseBusDelay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

/*********      Public function implementation    *********/

void I2C_Init() {
	if (I2C_Initialized) {
		return;
	}
	I2C_ReleaseBus();
	I2C_InitPins();
	I2C_MasterInit(I2C0, &I2C_Config, 60000000);
	I2C_MasterTransferCreateHandle(I2C0, &I2C_Handler, I2C_CallBack, NULL);
	I2C_Initialized = true;
}

void I2C_Write(uint8_t deviceAddr, uint8_t* txData, uint8_t len) {
#ifdef I2C_WITH_RTOS
	while(I2C_InUse) {
		RTOS_DELAY_MIN();
	}
	I2C_InUse = true;
#endif
	I2C_MasterXfer.slaveAddress = deviceAddr;
	I2C_MasterXfer.subaddress = 0;
	I2C_MasterXfer.subaddressSize = 0;
	I2C_MasterXfer.direction = kI2C_Write;
	I2C_MasterXfer.data = txData;
	I2C_MasterXfer.dataSize = len;
	I2C_MasterXfer.flags = kI2C_TransferDefaultFlag;
	I2C_MasterTransferNonBlocking(I2C0, &I2C_Handler, &I2C_MasterXfer);
	while(!I2C_CompleteFlag);
	I2C_CompleteFlag = false;
#ifdef I2C_WITH_RTOS
	I2C_InUse = false;
#endif
}

void I2C_Read(uint8_t deviceAddr, uint8_t regAddr, uint8_t* rxData, uint8_t len) {
#ifdef I2C_WITH_RTOS
	while(I2C_InUse) {
		RTOS_DELAY_MIN();
	}
#endif
	I2C_MasterXfer.slaveAddress = deviceAddr;
	I2C_MasterXfer.subaddress = regAddr;
	I2C_MasterXfer.subaddressSize = 1U;
	I2C_MasterXfer.direction = kI2C_Read;
	I2C_MasterXfer.data = rxData;
	I2C_MasterXfer.dataSize = len;
	I2C_MasterXfer.flags = kI2C_TransferDefaultFlag;
	I2C_MasterTransferNonBlocking(I2C0, &I2C_Handler, &I2C_MasterXfer);
	while(!I2C_CompleteFlag);
	I2C_CompleteFlag = false;
#ifdef I2C_WITH_RTOS
	I2C_InUse = false;
#endif
}
