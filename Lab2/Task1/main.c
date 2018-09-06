/**
 * Lab2 Task1 for ECE4550 Fall 2018
 *
 * Blue LED status reflects status of a pushbutton between J2-10 and J4-10
 * J2-10 is GND
 * J4-10 is GPIO0
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-09-06
 */

#include "F2837xD_device.h"

Uint16 tmp;

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
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;     // enable pull-up resistor

    // disable red led
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;  // gpio output
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;    // output
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;     // enable pull-up resistor
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // pushbutton on gpio0
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;   // gpio input
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;     // input
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;     // enable pull-up resistor

    WdRegs.WDCR.all = 0x28;  // activate timer
    EDIS;

    // Execution Code
    GpioDataRegs.GPASET.bit.GPIO31 = 1;  // force led off
    while (1) {
        // Execution Code
        tmp = GpioDataRegs.GPADAT.bit.GPIO0;
        if (tmp == 0) {
            GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
        } else {
            GpioDataRegs.GPASET.bit.GPIO31 = 1;
        }

        // watchdog timer reset
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}
