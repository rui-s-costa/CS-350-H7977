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

/* Driver Header files */
#include <ti/drivers/Power.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);

// I2C Global Variables
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    {0x48, 0x0000, "11X"},
    {0x49, 0x0000, "116"},
    {0x41, 0x0001, "006"}};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
I2C_Handle i2c;
I2C_Transaction i2cTransaction;
UART_Handle uart;
Timer_Handle timer0;

// temperature variables
int heat;
int seconds;
int setTemp;
int16_t currentTemp;

// task scheduler
typedef struct task
{
    int state;                 // Task's current state
    unsigned long period;      // Task period
    unsigned long elapsedTime; // Time elapsed since last task tick
    int (*TickFct)(int);       // Task tick function
} task;

task tasks[2];

// constants
const unsigned char taskNum = 2;
const unsigned long taskTime = 100;
const unsigned long heatTime = 500;
const unsigned long buttonTime = 200;
const unsigned long tasksTime = 100;
// Setting timer variables
volatile unsigned TimerFlag = 0;
int timerCount = 0;

// Button notifications
int buttonUp = 0;
int buttonDown = 0;

// Enumeration: CheckButtonStates
// Description: This enum defines the different states for checking buttons.
enum CheckButtonStates
{
    CHECK_BUTTONS_IDLE,      // Represents the idle state, waiting for button processing to start.
    CHECK_BUTTONS_PROCESSING // Represents the state where buttons are being processed.
};

int handleTemperatureButtons(int state);

// Enumeration: SetHeatStates
// Description: This enum defines the different states for the heat control state machine.
enum SetHeatStates
{
    SET_HEAT_IDLE,      // Represents the idle state, waiting for heat control to start.
    SET_HEAT_PROCESSING // Represents the state where heat control is being processed.
};

int controlHeatBasedOnTemperature(int state);

int16_t readTemp(void)
{

    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
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
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{

    unsigned char i;
    for (i = 0; i < taskNum; ++i)
    {
        if (tasks[i].elapsedTime >= tasks[i].period)
        {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += tasksTime;
    }

    TimerFlag = 1;
}

// initialize Timer
void initTimer(void)
{
    Timer_Params params;

    // init the driver
    Timer_init();

    // Config driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL)
    {
        // Failed to initialized
        while (1)
        {
        }
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        // Failed to start
        while (1)
        {
        }
    }
}

// Function: initUART
// Description: This function initializes the UART (Universal Asynchronous Receiver/Transmitter) communication module.
//              It configures the UART driver with specific parameters and opens the driver for communication.
void initUART(void)
{
    UART_Params uartParams;

    // Initialize the UART driver.
    UART_init();

    // Configure the UART driver with specific parameters.
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;  // Set the write data mode to binary (raw data).
    uartParams.readDataMode = UART_DATA_BINARY;   // Set the read data mode to binary (raw data).
    uartParams.readReturnMode = UART_RETURN_FULL; // Set the read return mode to return a full buffer when data is available.
    uartParams.baudRate = 115200;                 // Set the baud rate to 115200 bits per second.

    // Open the UART driver for communication using the specified parameters.
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL)
    {
        // UART_open() failed. In case of an error, the function enters an infinite loop to prevent further execution.
        while (1)
            ;
    }
}

// Function: initI2C
// Description: This function initializes the I2C (Inter-Integrated Circuit) communication module and detects the presence of temperature sensors on the I2C bus.
//              It configures the I2C driver with specific parameters, opens the driver for communication, and detects the presence of temperature sensors
//              by performing I2C transactions to their respective addresses. If a temperature sensor is found, its I2C address is displayed; otherwise,
//              an appropriate message is displayed.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    // Display an informational message indicating that the I2C driver is being initialized.
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Initialize the I2C driver.
    I2C_init();

    // Configure the I2C driver with specific parameters.
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;             // Set the I2C bit rate to 400 kHz.
    i2cParams.transferMode = I2C_MODE_BLOCKING; // Set the I2C transfer mode to blocking.

    // Open the I2C driver for communication using the specified parameters.
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        // I2C_open() failed. Display an error message and enter an infinite loop to prevent further execution.
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1)
            ;
    }

    // Display a message indicating that the I2C driver initialization passed.
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Prepare the I2C transaction structure with appropriate buffer pointers and counts for reading temperature sensor data.
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    // Iterate through the list of temperature sensors and attempt to detect their presence using I2C transactions.
    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        // Display a message asking if a temperature sensor with a specific ID is present.
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        // Perform an I2C transaction to check if the temperature sensor responds at the given address.
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            // If the sensor responds, display a message indicating that it was found.
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }

        // If the sensor does not respond, display a message indicating that it was not found.
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    // Display the result of the detection process.
    if (found)
    {
        // If a temperature sensor was found, display its ID and I2C address.
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        // If no temperature sensor was found, display an appropriate message.
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

// Function: controlHeatBasedOnTemperature
// Description: This function is responsible for controlling the heating element based on the current temperature and the desired set temperature.
//              It receives the current state as input and returns the updated state after processing the heat control logic.
// Parameters:
//   - int state: The current state of the heat control state machine.
int controlHeatBasedOnTemperature(int state)
{
    // State transition logic for the heat control state machine.
    switch (state)
    {
    case SET_HEAT_IDLE:
        // Move to the SET_HEAT_PROCESSING state to begin heat control logic.
        state = SET_HEAT_PROCESSING;
        break;
    case SET_HEAT_PROCESSING:
        // Stay in the SET_HEAT_PROCESSING state to continue heat control logic.
        state = SET_HEAT_PROCESSING;
        break;
    default:
        // In case of an unknown state, reset the state to SET_HEAT_PROCESSING to restart heat control logic.
        state = SET_HEAT_PROCESSING;
        break;
    }

    // Heat control logic in the SET_HEAT_PROCESSING state.
    switch (state)
    {
    case SET_HEAT_PROCESSING:
        // If the 'setTemp' is less than or equal to the 'currentTemp', turn off the heat (heat = 0) and turn on the LED.
        if (setTemp <= currentTemp)
        {
            heat = 0;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        }
        else
        {
            // If the 'setTemp' is greater than the 'currentTemp', turn on the heat (heat = 1) and turn off the LED.
            heat = 1;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        }

        break;
    }

    // Return the updated state after processing the heat control logic.
    return state;
}

// Function: handleTemperatureButtons
// Description: This function is responsible for handling temperature adjustments based on button presses.
//              It receives the current state as input and returns the updated state after processing button inputs.
// Parameters:
//   - int state: The current state of the button handling state machine.

int handleTemperatureButtons(int state)
{
    // State transition logic for the button handling state machine.
    switch (state)
    {
    case CHECK_BUTTONS_IDLE:
        // Move to the CHECK_BUTTONS_PROCESSING state to begin processing button inputs.
        state = CHECK_BUTTONS_PROCESSING;
        break;

    case CHECK_BUTTONS_PROCESSING:
        // Stay in the CHECK_BUTTONS_PROCESSING state to continue processing button inputs.
        state = CHECK_BUTTONS_PROCESSING;
        break;

    default:
        // In case of an unknown state, reset the state to CHECK_BUTTONS_IDLE to restart button processing.
        state = CHECK_BUTTONS_IDLE;
        break;
    }

    // Button processing in the CHECK_BUTTONS_PROCESSING state.
    switch (state)
    {
    case CHECK_BUTTONS_PROCESSING:
        // If the 'buttonUp' variable is greater than 0 and 'setTemp' is less than 99, increment the 'setTemp'.
        if (buttonUp > 0 && setTemp < 99)
        {
            setTemp += 1;
        }

        // If the 'buttonDown' variable is greater than 0 and 'setTemp' is greater than 0, decrement the 'setTemp'.
        if (buttonDown > 0 && setTemp > 0)
        {
            setTemp -= 1;
        }

        // Reset the buttonUp and buttonDown variables after processing the button inputs.
        buttonUp = 0;
        buttonDown = 0;

        break;
    }

    // Return the updated state after processing button inputs.
    return state;
}

// Function: gpioButtonFxn0
// Description: This function is called when a button press is detected. It increments the variable buttonUp by 1,
//              which is used to keep track of how many times the button has been pressed.
void gpioButtonFxn0()
{
    // Increment the variable buttonUp by 1 to keep track of button presses.
    buttonUp += 1;
}

// Function: gpioButtonFxn1
// Description: This function is called when a button press is detected. It increments the variable buttonDown by 1,
//              which is used to keep track of how many times the button has been pressed.
void gpioButtonFxn1()
{
    // Increment the variable buttonDown by 1 to keep track of button presses.
    buttonDown += 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init(); // Initialize the GPIO driver.
    initUART();  // Initialize the UART communication module.
    initI2C();   // Initialize the I2C communication module and detect temperature sensors.
    initTimer(); // Initialize the Timer for periodic tasks.

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);         // Configure LED pin as an output and set it to low (off).
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING); // Configure Button pin as an input with pull-up and interrupt on falling edge (button press).

    /* Turn on LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // Turn on the LED.

    /* Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0); // Set the callback function for Button0 (when it is pressed).

    /* Allows interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0); // Enable interrupts for Button0.

    /*
     *  If more than one input pin is available, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Config Button1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING); // Configure Button1 pin as an input with pull-up and interrupt on falling edge (button press).

        /* Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1); // Set the callback function for Button1 (when it is pressed).
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);                   // Enable interrupts for Button1.
    }

    unsigned long seconds;
    unsigned char i = 0;

    tasks[i].state = CHECK_BUTTONS_IDLE; // Set the initial state for the button checking task.
    tasks[i].period = buttonTime;        // Set the task period for button checking.
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &handleTemperatureButtons; // Set the function pointer to the button checking task function.
    ++i;
    tasks[i].state = SET_HEAT_IDLE; // Set the initial state for the heat control task.
    tasks[i].period = heatTime;     // Set the task period for heat control.
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &controlHeatBasedOnTemperature; // Set the function pointer to the heat control task function.

    heat = 0;     // Initialize the heat variable to 0 (heat off).
    seconds = 0;  // Initialize the seconds counter to 0.
    setTemp = 24; // Set the initial temperature set point to 24 degrees Celsius.

    // Loop forever
    while (1)
    {
        // Read current temperature
        currentTemp = readTemp(); // Perform the temperature reading and store the result in 'currentTemp'.

        // Timer
        while (!TimerFlag)
        {
        }
        TimerFlag = 0;
        seconds++; // Increment the seconds counter.

        // Display the temperature, set temperature, heat state, and seconds since the start in the specified format.
        DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", currentTemp, setTemp, heat, seconds))
    }

    return (NULL);
}
