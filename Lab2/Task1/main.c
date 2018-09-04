/**
 * Lab2 Task1 for ECE4550 Fall 2018
 *
 * Blue LED status reflects status of a pushbutton between J2-10 and J4-10
 *
 * Author: Marion Anderson,
 * Date: 2018-09-06
 */
#include "F2837xD_device.h"

int main(void)
{
    EALLOW;
    // set up gpio
    // See p943 for example mux bit layout
    // GpioCtrlRegs.GP?DIR.bit.GPIO?  // Direction settings: 0->input; 1->output
    // GpioCtrlRegs.GP?PUD.bit.GPIO?  // Pull-up/down settings
    WdRegs.WDCR.all = 0x68;
    WdRegs.WDCR.all = 0x28;
    EDIS;

    while (1) {
        // GpioDataRegs.GP?DAT.bit.GPIO?  // Read from GPIO
        // GpioDataRegs.GP?SET.bit.GPIO?  // Write to GPIO

        // watchdog timer reset
        EALLOW;
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
        EDIS;
    }
}
