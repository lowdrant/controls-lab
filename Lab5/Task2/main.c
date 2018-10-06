/**
 * Lab5 Task2 for ECE4550 Fall 2018
 *
 * Goal:
 * Control a bidirectional DC motor using the PWM from Task1.
 * Read the rotation of the motor from an encoder on J14 (QEP1).
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-10-04
 */

#include "F2837xD_device.h"
#ifndef REVTICKS
#define REVTICKS 360
#endif

// data loggers
vavg = 20;              // track desired avg voltage
float vodes[2000];      // log desired average voltage
float theta[2000];      // log actual angular position

// interrupt parameters
Uint16 state = 0;       // Vo state tracker
Uint32 tmr0_count = 0;  // software clock divider for state machine
interrupt void TimerISR(void);


int main(void)
{
    //==============================Begin Setup==============================
    DINT; EALLOW; WdRegs.WDCR.all = 0x68;

    // Clocking (fclk = 100 MHz)
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 1;   // external source (10 MHz)
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;       // disable PLL to edit
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // clear division register
    ClkCfgRegs.SYSPLLMULT.all = 20;               // clock multiplier (p.334)
    while (ClkCfgRegs.SYSPLLSTS.bit.LOCKS != 1);  // wait for PLL lock
    ClkCfgRegs.SYSCLKDIVSEL.all = 1;              // 1+desired division
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;       // make PLL system clock
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // final clock division
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;   // fclk,pwm = 100MHz

    // Canary LED
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;
    GpioDataRegs.GPASET.bit.GPIO31 = 1;  // turn off led so know code works

    // PWM1 Setup: Booster Pack Phase A Control
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    // clock
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;    // module clk EN
    asm(" NOP"); asm(" NOP");            // critical wait
    EPwm1Regs.TBCTL.bit.CTRMODE = 0b10;  // up-down count mode
    EPwm1Regs.TBPRD = 2000;              // vres = 24mV = 2VDC/TBPRD = 48V/2000
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;   // no further clock division needed
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;      // fpwm = fclk,pwm / (2*TBPRD*CLKDIV*HSPCLKDIV)
    // pulse control | D~=0.9167 (20V from +/-24V pulses)
    EPwm1Regs.CMPA.bit.CMPA = 166;       // D=0.9173 with maxval=2000
    EPwm1Regs.AQCTLA.bit.CAU = 0b10;     // 1A high on upcount
    EPwm1Regs.AQCTLA.bit.CAD = 0b01;     // 1A low on downcount
    EPwm1Regs.AQCTLB.bit.CAU = 0b01;     // 1B low on upcount
    EPwm1Regs.AQCTLB.bit.CAD = 0b10;     // 1B high in downcount

    // PWM2 Setup: Booster Pack Phase B Control
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;  // clock setup
    asm(" NOP"); asm(" NOP");
    EPwm2Regs.TBCTL.bit.CTRMODE = 0b10;
    EPwm2Regs.TBPRD = 2000;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.CMPA.bit.CMPA = 166;     // pulse control - opposite of PWM1
    EPwm2Regs.AQCTLA.bit.CAU = 0b01;
    EPwm2Regs.AQCTLA.bit.CAD = 0b10;
    EPwm2Regs.AQCTLB.bit.CAU = 0b10;
    EPwm2Regs.AQCTLB.bit.CAD = 0b01;

    // QEP Setup
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1; asm(" NOP"); asm(" NOP");
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;  // MAXIMUM COUNTER!!
    EQep1Regs.QPOSINIT = 0;          // init value of 0
    EQep1Regs.QEPCTL.bit.QPEN = 1;   // enable qep1
    EQep1Regs.QEPCTL.bit.SWI = 1;    // activate counter

    // Timer0 Interrupt (ftmr=1kHz)
    CpuTimer0Regs.TCR.bit.TSS = 1;      // stop timer0
    CpuTimer0Regs.PRD.all = 200e3 - 1;  // 200Mhz->1kHz
    CpuTimer0Regs.TCR.bit.TRB = 1;      // load timer division
    CpuTimer0Regs.TCR.bit.TIE = 1;      // Timer Interrupt Enable
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.TIMER0_INT = &TimerISR;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER = 1;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // system clock to pwm
    CpuTimer0Regs.TCR.bit.TSS = 0; WdRegs.WDCR.all = 0x28; EDIS; EINT;
    //===============================End Setup===============================

    while (1) {
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}


/** TimerISR
 *
 * 2 Tasks:
 * - Control voltage according to state machine
 * - Read position
 *
 *
 * Voltage Control
 * - Vo goes: +20V -> 0V -> -20V -> 0V -> +20V
 * - Vin is 24VDC
 * - Vo changes every 0.2s
 * - Vo needs to be between phases A and B on the booster pack
 *      ____                ____
 * ____|    |____      ____|    |____
 *               |____|
 *
 * PWM Control
 * - Booster pack is in 6-input mode
 * - Need to control 4 outputs (2 PWMs) for 2-phase output
 * - Must use +/-24V bursts (no 0V output mode that allows current flow)
 *
 *      PWM1A -> INHA
 *      PWM1B -> INLA
 *      PWM2A -> INHB
 *      PWM2B -> INLB
 *
 * Position Reading
 * - Read EQep1Regs.QPOSCNT to get how many ticks
 * - Convert ticks to angle
 *
 * The switch-case body of the code looks a little weird/lopsided because
 * on each interrupt it reads the state, changes the PWM, and then increments
 * to the next state. This is so we are ready to go on the next interrupt.
 *
 * Remember to subtract 1 from the <state> to get the current state when
 * debugging.
 *
 */
interrupt void TimerISR(void)
{
    // logic for switching state
    // switch state every 200 timer0 interrupts (1kHz -> 0.2s)
    if (tmr0_count % 200 == 0) {
        switch (state) {

            // +20V
            case 0:
                EPwm1Regs.AQCTLA.bit.CAU = 0b10;  // 1A high on upcount
                EPwm1Regs.AQCTLA.bit.CAD = 0b01;  // 1A low on downcount
                EPwm1Regs.AQCTLB.bit.CAU = 0b01;  // 1B low on upcount
                EPwm1Regs.AQCTLB.bit.CAD = 0b10;  // 1B high in downcount
                EPwm2Regs.AQCTLA.bit.CAU = 0b01;  // 2A low on upcount
                EPwm2Regs.AQCTLA.bit.CAD = 0b10;  // 2A high on downcount
                EPwm2Regs.AQCTLB.bit.CAU = 0b10;  // 2B high on upcount
                EPwm2Regs.AQCTLB.bit.CAD = 0b01;  // 2B low on downcount

                vavg = 20;
                state = 1;  // update state
                break;

            // 0V
            case 1:
                // force everything low for High-Z mode
                EPwm1Regs.AQCTLA.bit.CAU = 1;
                EPwm1Regs.AQCTLA.bit.CAD = 1;
                EPwm1Regs.AQCTLB.bit.CAU = 1;
                EPwm1Regs.AQCTLB.bit.CAD = 1;
                EPwm2Regs.AQCTLA.bit.CAU = 1;
                EPwm2Regs.AQCTLA.bit.CAD = 1;
                EPwm2Regs.AQCTLB.bit.CAU = 1;
                EPwm2Regs.AQCTLB.bit.CAD = 1;
                vavg = 0;
                state = 2;
                break;

            // -20VDC
            case 2:
                // mirror of case 0
                EPwm1Regs.AQCTLA.bit.CAU = 0b01;
                EPwm1Regs.AQCTLA.bit.CAD = 0b10;
                EPwm1Regs.AQCTLB.bit.CAU = 0b10;
                EPwm1Regs.AQCTLB.bit.CAD = 0b01;
                EPwm2Regs.AQCTLA.bit.CAU = 0b10;
                EPwm2Regs.AQCTLA.bit.CAD = 0b01;
                EPwm2Regs.AQCTLB.bit.CAU = 0b01;
                EPwm2Regs.AQCTLB.bit.CAD = 0b10;
                vavg = -20;
                state = 3;
                break;

            // 0VDC
            case 3:
                // force everything low for High-Z mode
                EPwm1Regs.AQCTLA.bit.CAU = 1;
                EPwm1Regs.AQCTLA.bit.CAD = 1;
                EPwm1Regs.AQCTLB.bit.CAU = 1;
                EPwm1Regs.AQCTLB.bit.CAD = 1;
                EPwm2Regs.AQCTLA.bit.CAU = 1;
                EPwm2Regs.AQCTLA.bit.CAD = 1;
                EPwm2Regs.AQCTLB.bit.CAU = 1;
                EPwm2Regs.AQCTLB.bit.CAD = 1;
                vavg = 0;
                state = 0;
                break;
        }
    }

    // Log data
    // use tmr0_count as the indexer for data
    if (tmr0_count < 2000) {
        vodes[tmr0_count] = vavg;
        theta[tmr0_count] = EQep1Regs.QPOSCNT / (float) REVTICKS;l
    }

    tmr0_count++;
    PieCtrlRegs.PIEACK.all = M_INT1;
}
