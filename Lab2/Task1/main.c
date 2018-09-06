/**
 * Lab2 Task1 for ECE4550 Fall 2018
 *
 * Blue LED status reflects status of a pushbutton between J2-10 and J4-10
 * J2-10 is GND
 * J4-10 is GPIO0
 *
 * Author: Marion Anderson,
 * Date: 2018-09-06
 */
#include "F2837xD_device.h"

int main(void)
{
    // Configure MCU
    EALLOW;
    WdRegs.WDCR.all = 0x68;  // disable timer

    // GPIO Setup
    // blue LED is gpio31 in port A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;  // gpio output
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;    // output

    // pushbutton on gpio0
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;   // gpio input
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;     // input
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;     // enable pull-up resistor

    WdRegs.WDCR.all = 0x28;  // activate timer
    EDIS;

    // Execution Code
    int state = 1;       // button state (0->L, 1->H)
    int led_state = 1;   // led pin state (1->H->off, 0->L->on)
    GpioDataRegs.GPASET.bit.GPIO31 = led_state;  // force led off
    while (1) {
        // Pushbutton control
        // when pin goes low, button is pressed (because pull-up)
        state = GpioDataRegs.GPADAT.bit.GPIO31;
        if (state == 0) {
            led_state++;     // increment led_state to switch mod2 state
            led_state %= 2;  // mod 2 constrains to 0, 1
            GpioDataRegs.GPASET.bit.GPIO31 = led_state;  // set LED to led_state
        }


        // watchdog timer reset
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}
