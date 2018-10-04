/**
 * Lab5 Task1 for ECE4550 Fall 2018
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-10-04
 */

#include "F2837xD_device.h"
#include "math.h"
#ifndef PI
#define PI 3.1415
#endif
#ifndef VREF
#define VREF 3
#endif

//float theta;  // motor initial position
float vavg = 20;        // desired average voltage on PWM
Uint16 state = 0;       // state tracker
Uint32 tmr0_count = 0;  // software "clock divider" for switching state
Uint32 ct = 0;
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
    // fclk,pwm = 100MHz
    // fpwm = 25kHz
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;  // 200MHz / 2 = 100MHz
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;            // module clk EN
    asm(" NOP"); asm(" NOP");                    // critical wait
    // MTRA
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;            // module clk EN
    EPwm1Regs.TBCTL.bit.CTRMODE = 0b10;  // up-down count mode
    EPwm1Regs.TBPRD = 2000;              // vres = 24mV = 2VDC/TBPRD = 48V/TBPRD
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;   // no further division required to reduce fpwm
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;      // fpwm = fclk,pwm / (2*TBPRD*CLKDIV*HSPCLKDIV)
    EPwm1Regs.CMPA.bit.CMPA = 340;
    EPwm1Regs.AQCTLA.bit.CAU = 0b10;
    EPwm1Regs.AQCTLA.bit.CAD = 0b01;

    // MTRB
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
   GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;            // module clk EN
    asm(" NOP"); asm(" NOP");
    EPwm2Regs.TBCTL.bit.CTRMODE = 0b10;
    EPwm2Regs.TBPRD = 2000;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.CMPA.bit.CMPA = 340;
    EPwm2Regs.AQCTLA.bit.CAU = 0b01;
    EPwm2Regs.AQCTLA.bit.CAD = 0b10;
    // Enable clock connection to all pwm
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;


    // QEP Setup
//    GpioCtrlRegs.GP?GMUX?.bit.GPIO? = ;
//    GpioCtrlRegs.GP?MUX?.bit.GPIO? = ;
//    CpuSysRegs.PCLKCR4.bit.EQEP? = ?;  // QEP CLK EN
//    asm(" NOP"); asm(" NOP");
//    EQep?Regs.QPOSMAX = 0xFFFFFFFF;  // MAXIMUM COUNTER
//    EQep?Regs.QPOSINIT = ;  // counter EN & init
//    EQep?Regs.QEPCTL.bit.QPEN = ;
//    EQep?Regs.QEPCTL.bit.SWI = ;

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
//    Uint32 j = 0;
//    for (j = 0; j < 1000; j++) {}
    CpuTimer0Regs.TCR.bit.TSS = 0;
    WdRegs.WDCR.all = 0x28; EDIS; EINT;
    //===============================End Setup===============================


    while (1) {
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
        ct++;
    }
}




/** vcalcISR
 *
 * Controls PWM to create desired average voltage from 24VDC supply
 * Computes next vavg from state machine:
 *      +20V -> 0V -> -20V -> 0V -> +20V
 *
 * Timer0 interrupt
 * ftmr0 = 1kHz
 *
 */
interrupt void vcalcISR(void)
{
    // logic for switching state
    // switch state every 200 clock triggers (1kHz -> 0.2s)
    if (tmr0_count % 200 == 0) {
        switch (state) {

            // +20VDC out
            case 0:
                // 83% duty cycle from 24VDC in
                vavg = 20;
                // PWM2 forced low in prev state, so don't set here
//                EPwm1Regs.CMPA.bit.CMPA = 340;  // 83% coverage of TBPRD=2000
//                GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
                // update state
                state = 1;
                break;

            // 0VDC
            case 1:
                // 0% duty cycle
                vavg = 0;
//                EPwm1Regs.CMPA.bit.CMPA = 0;  // 100% coverage of TBPRD=2000
//                EPwm2Regs.CMPA.bit.CMPA = 0;  // 100% coverage of TBPRD=2000

                // update state
                state = 2;
                break;

            // -20VDC
            case 2:
                // 0% duty cycle
                vavg = -20;
                // PWM1 forced low in prev state, so don't set here
//                EPwm1Regs.CMPA.bit.CMPA = 0;  // 83% coverage of TBPRD=2000
//                EPwm2Regs.CMPA.bit.CMPA = 340;  // 83% coverage of TBPRD=2000

                // update state
                state = 3;
                break;

            // 0VDC
            case 3:
                // 0% duty cycle
                vavg = 0;
//                EPwm1Regs.CMPA.bit.CMPA = 0;  // 83% coverage of TBPRD=2000
//                EPwm2Regs.CMPA.bit.CMPA = 0;  // 100% coverage of TBPRD=2000

                // update state
                state = 0;
                break;
        }
    }

    tmr0_count++;
    PieCtrlRegs.PIEACK.all = M_INT1;
}
