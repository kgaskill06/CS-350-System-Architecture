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
#include <ti/drivers/Timer.h>

volatile unsigned char TimerFlag = 0;

//Controller button flags
volatile int16_t L_BUTTON = 0;
volatile int16_t R_BUTTON = 0;
volatile uint_least8_t button;

/*
 * Struct for tasks timer callback of state
 * Variables for period of light blinking 3500 ms for SOS. building own from scratch to include state
 *
 */
typedef struct task {

    int stateTask;
    unsigned long periodTime;
    unsigned long elapsedTime;
    int (*TickFct) (int);
} task;

task tasks[2];


const unsigned char taskNum = 2;
const unsigned long tasksPeriodGCD = 500;
const unsigned long periodSOS = 1500;
const unsigned long periodOK = 500;

enum GPIO_States { GPIO_SMStart, LIGHT_OFF, GREEN_LIGHT_OFF, RED_LIGHT_OFF, MORSE_SOS, GPIO_SOS, GPIO_OK, MOORSE_OK, GPIO_TOGGLE0, GPIO_TOGGLE1} state;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    unsigned int i;
    TimerFlag = 1; // isr flag
    for (i = 0; i < taskNum; ++i) {

        if (tasks[i].elapsedTime >= tasks[i].periodTime) {
            tasks[i].stateTask = tasks[i].TickFct(tasks[i].stateTask);
            tasks[i].elapsedTime = 0;
        }

        tasks[i].elapsedTime += tasksPeriodGCD;

    }



}

void initTimer(void) {
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
Timer_Params_init(&params);
params.period = 500000; // changed from 1000000
params.periodUnits = Timer_PERIOD_US;
params.timerMode = Timer_CONTINUOUS_CALLBACK;
params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
       /* Failed to initialized timer */
       while (1) {

       }
     }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {

        }
    }
}

G

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);
    L_BUTTON = 1;


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
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1);
    R_BUTTON = 1;
}

//tick function, put logic for counters
TickFct_SOS() {
    unsigned int i;
    unsigned int S_counter;
    unsigned int counter;

    switch(state) {
    /*case GPIO_SMStart:
        if(L_BUTTON == 1) {
            state = GPIO_TOGGLE0;
        }

        else if(R_BUTTON == 1) {
            state = GPIO_TOGGLE1;
        }
        break; commenting out to rework state*/

    case GPIO_SOS:
        state = MORSE_SOS;

        break;
    }

    switch(state) {
    case GPIO_SOS:
        GPIO_toggle(CONFIG_GPIO_LED_0);
        GPIO_toggle(CONFIG_GPIO_LED_1);
        for (i = 0; i < 3; ++i) {
            if (!(i < 3)) {
                i = 0;
                state = GREEN_LIGHT_OFF;
            }
        }
        break;


    case GPIO_TOGGLE1:
        GPIO_toggle(CONFIG_GPIO_LED_1);
        if (S_counter == 0) {
            if (counter < 6) {
                GPIO_toggle(CONFIG_GPIO_LED_0);
                counter++;
            }
        /*for (i = 0; i < 6; ++i) {
            GPIO_toggle(CONFIG_GPIO_LED_1);
            count++; */

        }
        break;

    case LIGHT_OFF:
        GPIO_toggle(CONFIG_LED_OFF);

        break;

    case RED_LIGHT_OFF:
        // do stuff
        break;

    case GREEN_LIGHT_OFF:
        // do stuff
        break;
    }
}


TickFct_OK() {
    unsigned int i;
    unsigned int S_counter;
    unsigned int counter;

    //switch(state) {
    /*case GPIO_OK:
        if(L_BUTTON == 1) {
            state = GPIO_TOGGLE0;
        }

        else if(R_BUTTON == 1) {
            state = GPIO_TOGGLE1;
        }
        break;

    case GPIO_SOS:
        state = MORSE_OK;

        break;
    } commenting out to rework state design*/

    switch(state) {
    case GPIO_TOGGLE0:
        GPIO_toggle(CONFIG_GPIO_LED_0);
        GPIO_toggle(CONFIG_GPIO_LED_1);
        for (i = 0; i < 3; ++i) {
            if (!(i < 3)) {
                i = 0;
                state = GREEN_LIGHT_OFF;
            }
        }
        break;


    case GPIO_SOS:
        GPIO_toggle(CONFIG_GPIO_LED_1);
        if (S_counter == 0) {
            if (counter < 6) {
                GPIO_toggle(CONFIG_GPIO_LED_0);
                counter++;
            }
        /*for (i = 0; i < 6; ++i) {
            GPIO_toggle(CONFIG_GPIO_LED_1);
            count++; */

        }
        break;
    }
}
    /* case LIGHT_OFF:
        GPIO_toggle(CONFIG_LED_OFF);

        break;

    case RED_LIGHT_OFF:
        // do stuff
        break;

    case GREEN_LIGHT_OFF:
        // do stuff
        break;
    } */



/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
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
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }


    // |**reworking how state machine passes operate**| state = GPIO_SMStart;
    unsigned char i = 0;

    tasks[i].stateTask = GPIO_SOS;
    tasks[i].periodTime = periodSOS;
    tasks[i].elapsedTime = tasks[i].periodTime;
    tasks[i].TickFct = &TickFct_SOS;
    //++i;
    tasks[i].stateTask = GPIO_OK;
    tasks[i].periodTime = periodOK;
    tasks[i].elapsedTime = tasks[i].periodTime;
    tasks[i].TickFct = &TickFct_OK;

    TimerSet(tasksPeriodGCD);
    TimerOn();

    while (1) {
        TickFct_SOS();
        while (!TimerFlag){}
    }

    //return state;
}

   /*      while (1) {

            GPIO_toggle(&button);

            switch(button) {

            case gpioButtonFxn0:
                state = GPIO_TOGGLE0;
                break;

            case gpioButtonFxn1:
            state = GPIO_TOGGLE1;
            break;

            }

            switch(state) {

            case GPIO_TOGGLE0:
                GPIO_toggle(CONFIG_GPIO_LED_0);
                break;


            case GPIO_TOGGLE1:
                GPIO_toggle(CONFIG_GPIO_LED_1);
                break;
        } */
