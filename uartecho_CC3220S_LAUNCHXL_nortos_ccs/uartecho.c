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
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Defines the states for the state machine */
#define SERSTATE_INITIAL    0
#define SERSTATE_O          1
#define SERSTATE_N          2
#define SERSTATE_F          3
#define SERSTATE_SECONDF    4

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    char        input;
    const char  echoPrompt[] = "Echoing characters:\r\n";
    UART_Handle uart;
    UART_Params uartParams;

    /*Declares the variables for the input for the state machine */
    unsigned char serReadState = 0;
    unsigned char ledState = 0;

    /* Call driver init functions */
    GPIO_init();
    UART_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART_write(uart, echoPrompt, sizeof(echoPrompt));

    /* Loop forever echoing */
    while (1) {
        /* Reads the input from the terminal */
        UART_read(uart, &input, 1);

        /* Switch statement for the state machine reading the input. It is in the loop to constantly read the input. It switches the state based on the input. */
        switch(serReadState) {
            case SERSTATE_INITIAL:
                if(input == 'O') {
                    serReadState = SERSTATE_O;
                } else {
                    serReadState = SERSTATE_INITIAL;
                }
            break;

            case SERSTATE_O:
                if(input == 'N') {
                    serReadState = SERSTATE_N;
                } else if(input == 'F') {
                    serReadState = SERSTATE_F;
                } else {
                    serReadState = SERSTATE_INITIAL;
                }
            break;

            case SERSTATE_N:
                /* This state means that ON has been met */
                ledState = 1;
                serReadState = SERSTATE_INITIAL;
                break;

            case SERSTATE_F:
                if(input == 'F') {
                    serReadState = SERSTATE_SECONDF;
                } else {
                    serReadState = SERSTATE_INITIAL;
                }
                break;

            case SERSTATE_SECONDF:
                /* This state means that Off has been met */
                ledState = 0;
                serReadState = SERSTATE_INITIAL;
                break;

            default:
                serReadState = SERSTATE_INITIAL;
                break;
            }

        /* Switch statement to control the LED. CONFIG_GPIO_LED_OFF turns the LED off and CONFIG_GPIO_LED_ON turns the LED on. */
        switch(ledState) {
            case 0:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                break;
            case 1:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                break;
            default:
                break;
        }
    }
}
