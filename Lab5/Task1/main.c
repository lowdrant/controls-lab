/**
 * Lab5 Task1 for ECE4550 Fall 2018
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-10-04
 */

#include "F2837xD_device.h"
#include "F2837xD_pievect.h"
#include "math.h"
#ifndef PI
#define PI 3.1415
#endif
#ifndef VREF
#define VREF 3
#endif

float theta;  // motor initial position
float vavg;   // desired average voltage on PWM
interrupt void vcalcISR(void);


int main(void)
{
    //==============================Begin Setup==============================
    DINT; EALLOW; WdRegs.WDCR.all = 0x68;

    // Test
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;
    GpioDataRegs.GPASET.bit.GPIO31 = 1;  // turn off led so know code works

    // Clocking (fclk = 100 MHz)
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 1;   // external source (10 MHz)
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;       // disable PLL to edit
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // clear division register
    ClkCfgRegs.SYSPLLMULT.all = 20;               // clock multiplier (p.334)
    while (ClkCfgRegs.SYSPLLSTS.bit.LOCKS != 1);  // wait for PLL lock
    ClkCfgRegs.SYSCLKDIVSEL.all = 1;              // 1+desired division
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;       // make PLL system clock
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // final clock division

    // PWM
    // fpwm = 100MHz
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;  // 200MHz / 2 = 100MHz
    CpuSysRegs.PCLKCR2.bit.EPWM? = 1;            // module clk EN
    asm(" NOP"); asm(" NOP");                    // critical wait
    // TODO: use xbar? (p. 1682)
    EPwm?Regs.TBCTL.bit.CTRMODE = 0;  // up-count mode
    EPwm?Regs.TBPRD = 2000;           // 2VDC/TBPRD = vres = 24mV = 48V/TBPRD
    EPwm?Regs.TBCTL.bit.HSPCLKDIV = ;
    EPwm?Regs.TBCTL.bit.CLKDIV = ;

    // QEP Setup
    GpioCtrlRegs.GP?GMUX?.bit.GPIO? = ;
    GpioCtrlRegs.GP?MUX?.bit.GPIO? = ;
    CpuSysRegs.PCLKCR4.bit.EQEP? = ?;  // QEP CLK EN
    asm(" NOP"); asm(" NOP");
    EQep?Regs.QPOSMAX = 0xFFFFFFFF;  // MAXIMUM COUNTER
    EQep?Regs.QPOSINIT = ;  // counter EN & init
    EQep?Regs.QEPCTL.bit.QPEN = ;
    EQep?Regs.QEPCTL.bit.SWI = ;

    // Interrupt Timer (timer0, ftmr=1kHz)
    // from 200MHz to 1kHz
    CpuTimer0Regs.TCR.bit.TSS = 1;         // stop timer0
    CpuTimer0Regs.PRD.all = 200e3 - 1;     // 200Mhz->1kHz
    CpuTimer0Regs.TCR.bit.TRB = 1;         // load timer division
    CpuTimer0Regs.TCR.bit.TIE = 1;         // Timer Interrupt Enable

    // Interrupt Assignment
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.TIMER0_INT = &vcalcISR;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER = 1;

    // wait 500us at SYSCLK = 200MHz
    // (100000 cycles)
    Uint32 j = 0;
    for (j = 0; j < 1000; j++) {}
    CpuTimer0Regs.TCR.bit.TSS = 0; WdRegs.WDCR.all = 0x28; EDIS; EINT;
    //===============================End Setup===============================

    while (1) {
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}




/** vcalcISR
 *
 * Recalculates desired vavg
 *
 * Timer0 interrupt
 *
 */
interrupt void vcalcISR(void)
{
    vavg = 0;
}
