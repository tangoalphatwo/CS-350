/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/I2C.h>

#include <ti/drivers/Timer.h>

#include <ti/drivers/UART.h>
#define DISPLAY(x) UART_write(uart, &output, x);


// Driver Handles - Global variables
Timer_Handle timer0;

/* Flags */
volatile unsigned char TimerFlag = 0;
volatile unsigned char TimerCounter = 0;
volatile unsigned char TimerFlag200m = 0;
volatile unsigned char TimerFlag500m = 0;
volatile unsigned char TimerFlag1s = 0;
volatile unsigned int second = 0;

volatile unsigned char ButtonUpPushed = 0;
volatile unsigned char ButtonDownPushed = 0;

/* Declares functions */
void initGPIO(void);
void initTimer(void);
void initI2C(void);
int16_t readTemp(void);
void initUART(void);

void gpioButtonFxn0(uint_least8_t index);
void gpioButtonFxn1(uint_least8_t index);
void timerCallback(Timer_Handle, int_fast16_t);

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
{ 0x48, 0x0000, "11X" },
{ 0x49, 0x0000, "116" },
{ 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;
int16_t temperature;
volatile unsigned int TemperatureTarget = 25;
volatile unsigned char heatFlag = 0;

void *mainThread(void *arg0)
{
    initGPIO();     // Init GPIO for Button0, Button1 and Led0
    initTimer();    // Init Timer with 100msec interval
    initUART();     // Init UART to serial messages
    initI2C();      // Init I2C for temperature sensor

    while(1)
    {
        if (TimerFlag200m)              // If 200ms passed
        {
            TimerFlag200m = 0;          // Clear the flag

            // Every 200ms check the button flags
            if(ButtonUpPushed)          // If increase button pushed
            {
                ButtonUpPushed = 0;     // Clear the flag
                TemperatureTarget++;    // Increase Temperature Target
            }
            if(ButtonDownPushed)        // If decrease button pushed
            {
                ButtonDownPushed = 0;   // Clear the flag
                TemperatureTarget--;    // Decrease Temperature Target
            }
        }

        if (TimerFlag500m)                                              // If 500ms passed
        {
            TimerFlag500m = 0;                                          // Clear the flag
            /* Every 500ms read the temperature and update the LED */
            temperature = readTemp();                                   // Read temperature from integrated sensor
            if (temperature < TemperatureTarget)                        // If it is smaller than target value
            {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);      // Open the LED as heater
                heatFlag = 1;                                           // Set the informative flag
            }
            else if (temperature >= TemperatureTarget)                  // If it is smaller than target value
            {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);     // Close the LED as heater
                heatFlag = 0;                                           // Reset the informative flag
            }
        }

        if (TimerFlag1s)    // For every second
        {
            TimerFlag1s = 0;    // Clear the flag

            /* Every second output the following to the UART */
            DISPLAY(snprintf(output,64,"<%02d,%02d,%d,%04d>\n\r",temperature,TemperatureTarget,heatFlag,second))    // Inform user via UART communication
        }

        while (!TimerFlag) {}
        TimerFlag=0;

        /* ++TimerCounter */
        TimerCounter = (TimerCounter + 1) % 10; // counts up to 10. It makes it easier to keep both 200 and 500 ms.
        if (!(TimerCounter %2))                 // If it is divided by 2, It means another 200ms passed.
            TimerFlag200m = 1;                      // 200ms passed flag
        if (!(TimerCounter %5))                 // If it is divided by 5, It means another 500ms passed.
            TimerFlag500m = 1;                      // 500ms passed flag
        if (!TimerCounter)                      // If it is divided by 10, It means another 500ms passed.
        {
            TimerFlag1s = 1;                        // 1 second passed flag
            second = (second+1)%10000;
        }
    }
}

void initGPIO(void)
{
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000; // 100ms period
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
    /* Failed to initialized timer */
    while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
    /* Failed to start timer */
    while (1) {}
    }
}

void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
        break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found,contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
    /*
    * Extract degrees C from the received data;
    * see TMP sensor datasheet
    */
    temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
    temperature *= 0.0078125;
    /*
    * If the MSB is set '1', then we have a 2's complement
    * negative value which needs to be sign extended
    */
    if (rxBuffer[0] & 0x80)
    {
    temperature |= 0xF000;
    }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor(%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

void initUART(void)
{
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

void gpioButtonFxn0(uint_least8_t index)
{
    ButtonUpPushed = 1;
}

void gpioButtonFxn1(uint_least8_t index)
{
    ButtonDownPushed = 1;
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;                          // 100ms passed flag
}
