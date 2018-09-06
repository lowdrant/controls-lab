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
Uint16 tmp;  // global declaration for debugger

int main(void)
{
    // Configure MCU
    EALLOW;
    WdRegs.WDCR.all = 0x68;  // disable timer

    // GPIO Setup
    // blue LED is gpio31 in port A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;  // mux to gpio
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;    // output mode
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;    // enable pull-up resistor to force LED off

    // disable red led
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;    // pull-up resistor to force LED off
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;    // force red off

    // pushbutton on gpio0
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;     // input mode
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;

    WdRegs.WDCR.all = 0x28;  // activate timer
    EDIS;

    // Execution Code
    GpioDataRegs.GPASET.bit.GPIO31 = 1;  // force led off
    while (1) {
        // Control LED
        // High state corresponds to "No action"
        // because the C2000 only has pull-up functionality,
        // so the default value is High
        tmp = GpioDataRegs.GPADAT.bit.GPIO0;  // store register data in memory
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
