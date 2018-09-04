/**
 * Lab2 Task2 for ECE4550 Fall 2018
 *
 * Program runs in Flash; Red LED toggles every 10^5 iterations
 *
 * Author: Marion Anderson,
 * Date: 2018-09-06
 */
#include "F2837xD_device.h"
#include "string.h"

// Code from prelab
extern Uint16 RamFuncs_loadstart;
extern Uint16 RamFuncs_loadsize;
extern Uint16 RamFuncs_runstart;
#pragma CODE_SECTION(InitFlash, "RamFuncs")
void InitFlash(void)
{
    EALLOW;
    Flash0CtrlRegs.FPAC1.bit.PMPPWR = 0x1;
    Flash0CtrlRegs.FBFALLBACK.bit.BNKPWR0 = 0x3;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.DATA_CACHE_EN = 0;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.PREFETCH_EN = 0;
    Flash0CtrlRegs.FRDCNTL.bit.RWAIT = 0x3;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.DATA_CACHE_EN = 1;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.PREFETCH_EN = 1;
    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0xA;
    EDIS;
    asm(" RPT #6 || NOP");
}


int main(void)
{
    // More from prelab
    // move program memory between flash and ram
    // put this at start of main
    memcpy(&RamFuncs_runstart, &RamFuncs_loadstart, (Uint32) &RamFuncs_loadsize);
    InitFlash();

    // Register setup
    EALLOW;
    WdRegs.WDCR.all = 0x68;
    WdRegs.WDCR.all = 0x28;
    // set up gpio
    // See p943 for example mux bit layout
    // GpioCtrlRegs.GP?DIR.bit.GPIO?  // Direction settings: 0->input; 1->output
    // GpioCtrlRegs.GP?PUD.bit.GPIO?  // Pull-up/down settings
    EDIS;

    // State variables
    unsigned long loop_count = 0;  // loop counter
    unsigned short led_state = 0;  // tracks LED on or off (off at first)
    while (1) {
        // check to toggle led
        if (!loop_count % 10000){
            led_state++;     // incrementing changes value in mod 2
            led_state %= 2;  // reset state to be either 0 or 1
        }
        loop_count++;

        // watchdog timer reset
        EALLOW;
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
        EDIS;
    }
}
