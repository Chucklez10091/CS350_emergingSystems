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
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART2_write(uart, x, sizeof(x), &bytesWritten);

volatile unsigned char buttonPressed = 0;
volatile unsigned char buttonDownFlag = 0;
volatile unsigned char buttonUpFlag = 0;
const uint32_t timerPeriod = 100000;
unsigned int seconds = 0;
volatile int16_t temperature = -1;
volatile unsigned char heat = 0;
volatile int16_t setpoint = 20;

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // raise buttonDown if other button not pressed
    if (!buttonPressed) buttonDownFlag = 1;
    // raise buttonPressed flag
    buttonPressed = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // raise buttonDown if other button not pressed
    if (!buttonPressed) buttonUpFlag = 1;
    // raise buttonPressed flag
    buttonPressed = 1;
}

enum CB_States {CB_SMStart, CB_SMNotPressed, CB_SMPressed} CB_State;
enum RT_States {RT_SMStart, RT_SMUnlit, RT_SMLit} RT_State;

/*
 *  ======== UART Driver ========
 */

// UART Global Variables
char  output[64];
size_t bytesWritten = 0;
int  bytesToSend;
// Driver Handles - Global variables
UART2_Handle uart;
void initUART(void) {
    UART2_Params uartParams;
    // Init the driver
    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}



/*
 *  ======== I2C driver ========
 */

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] = {
               { 0x48, 0x0000, "11X" },
               { 0x49, 0x0000, "116" },
               { 0x41, 0x0001, "006" }
};
uint8_t   txBuffer[1];
uint8_t   rxBuffer[2];
I2C_Transaction  i2cTransaction;

// Driver Handles - Global Variables
I2C_Handle  i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void) {
    int8_t  i, found;
    I2C_Params  i2cParams;
    DISPLAY("Initializing I2C Driver - ");
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY("Failed\n\r") while (1);
    }
    DISPLAY("Passed\n\r");

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount  = 0;
    found = false;
    for (i=0; i<3; ++i) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        int j = snprintf(output, 64, "Is this %s? ", sensors[i].id);
        DISPLAY(output);
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY("Found\n\r") found = true;
            break;
        }
        DISPLAY("No\n\r");
    }  if(found) {
        int j = snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress);
        DISPLAY(output);
    } else {
        DISPLAY("Temperature sensor not found, contact professor\n\r");
    }
}

int16_t readTemp(void) {
    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount  = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        /* * Extract degrees C from the received data; * see TMP sensor datasheet */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]); temperature *= 0.0078125;
        /* * If the MSB is set '1', then we have a 2's complement * negative value which needs to be sign extended */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        int j = snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status);
        DISPLAY(output);
        DISPLAY("Please power cycle your board by unplugging USB and plugging back in.\n\r");
    }
    return temperature;
}


/*
 *  ======== Timer Driver ========
 */

// Driver Handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}
void initTimer(void) {
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = timerPeriod;
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

/* Define task structure */
typedef struct task {
   int state;                  // Task's current state
   unsigned long period;       // Task period
   unsigned long elapsedTime;  // Time elapsed since last task tick
   int (*TickFct)(int);        // Task tick function
} task;

int TickFct_chkButton(int state){
    switch(state){
        case CB_SMStart:
            state = CB_SMNotPressed;
            break;
        case CB_SMNotPressed:
            // check button flags
            if (buttonPressed){
                state = CB_SMPressed;
            }
            break;
        case CB_SMPressed:
            if (!buttonPressed) state = CB_SMNotPressed;
            break;
        default:
            state = CB_SMStart;
            break;
    }
    switch(state){
        case CB_SMStart:
            break;
        case CB_SMNotPressed:
            break;
        case CB_SMPressed:
            // check button flags
            if (buttonPressed){
                if (buttonDownFlag){
                    setpoint--;
                    buttonDownFlag = 0;
                }
                if (buttonUpFlag){
                    setpoint++;
                    buttonUpFlag = 0;
                }
                buttonPressed = 0;
            }
            break;
        default:
            state = CB_SMStart;
            break;
    }
    return state;
}

int TickFct_readTemp(int state){
    temperature = readTemp();
    switch (state){
        case RT_SMStart:
            state = RT_SMUnlit;
            break;
        case RT_SMUnlit:
            if (temperature < setpoint) state = RT_SMLit;
            break;
        case RT_SMLit:
            if (temperature >= setpoint) state = RT_SMUnlit;
            break;
        default:
            break;
    }

    switch (state) {
        case RT_SMStart:
            break;
        case RT_SMUnlit:
            heat = 0;
            GPIO_write(CONFIG_GPIO_LED_0, 0);
            break;
        case RT_SMLit:
            heat = 1;
            GPIO_write(CONFIG_GPIO_LED_0, 1);
            break;
        default:
            state = RT_SMStart;
            break;
    }
    return state;
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    /*
     * Initialize Tasks
     */
    task CB_task;
    task RT_task;

    CB_task.state = CB_SMStart;
    CB_task.period = 200000;
    CB_task.elapsedTime = 200000;
    CB_task.TickFct = &TickFct_chkButton;

    RT_task.state = RT_SMStart;
    RT_task.period = 500000;
    RT_task.elapsedTime = 500000;
    RT_task.TickFct = &TickFct_readTemp;

    int j = snprintf(output, 64,"%64c", '\0');

    // loop forever
    while (1){
        // every 200ms (*1000us) check the button flags
        if (CB_task.elapsedTime >= CB_task.period){
            CB_task.state = CB_task.TickFct(CB_task.state);
            CB_task.elapsedTime = 0;
        }

        // every 500ms read the temperature and update the LED
        if (RT_task.elapsedTime >= RT_task.period){
            RT_task.state = RT_task.TickFct(RT_task.state);
            RT_task.elapsedTime = 0;
        }
        // Every 1000ms output to the UART
        if (seconds % (10 * timerPeriod) == 0){
            int j = snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds / (10*timerPeriod));
            DISPLAY(output);
        }

        // timer period config
        while (!TimerFlag){}
        TimerFlag = 0;
        CB_task.elapsedTime += timerPeriod;
        RT_task.elapsedTime += timerPeriod;
        seconds += timerPeriod;
    }

    return (NULL);
}
