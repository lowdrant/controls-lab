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

#ifndef VIN            // System Input Voltage
#define VIN 24
#endif
#ifndef PWM_MAX_COUNT  // PWM Maximum Count
#define PWM_MAX_COUNT 2000
#endif
#ifndef TICKS_PER_REV  // QEP Ticks per Revolution
#define TICKS_PER_REV 1000  // float to avoid integer div
#endif
#ifndef LOG_LENGTH     // data points in data loggers
#define LOG_LENGTH 2000
#endif

#include "F2837xD_device.h"
#include "math.h"

// Data Loggers
Uint16 i;                     // index of data arrays
int vo = 20;                  // track desired avg voltage
float vo_log[LOG_LENGTH];     // log desired average voltage
float theta_log[LOG_LENGTH];  // log actual angular position

// State Machine Control
Uint16 state = 0;       // Output voltage state tracker
Uint32 tmr0_count = 0;  // software "clock divider" for incrementing states
interrupt void TimerISR(void);  // timer0-based interrupt for lab i/o
Uint16 pwmCMP(float);           // pwm compare value calculator


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
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;    // module clk EN
    asm(" NOP"); asm(" NOP");            // critical wait
    EPwm1Regs.TBCTL.bit.CTRMODE = 0b10;  // up-down count mode
    EPwm1Regs.TBPRD = PWM_MAX_COUNT;     // vres = 24mV = 2VDC/TBPRD = 48V/2000
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;   // no further clock division needed
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;      // fpwm = fclk,pwm / (2*TBPRD*CLKDIV*HSPCLKDIV)
    // pulse control | D~=0.9167 (20V from +/-24V pulses)
    EPwm1Regs.CMPA.bit.CMPA = 167;       // D=0.9173 with maxval=2000
    EPwm1Regs.AQCTLA.bit.CAU = 0b10;     // 1A high on upcount
    EPwm1Regs.AQCTLA.bit.CAD = 0b01;     // 1A low on downcount
    EPwm1Regs.AQCTLB.bit.CAU = 0b01;     // 1B low on upcount
    EPwm1Regs.AQCTLB.bit.CAD = 0b10;     // 1B high in downcount

    // PWM2 Setup: Booster Pack Phase B Control
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0; GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;  // clock setup
    asm(" NOP"); asm(" NOP");
    EPwm2Regs.TBCTL.bit.CTRMODE = 0b10;
    EPwm2Regs.TBPRD = PWM_MAX_COUNT;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.CMPA.bit.CMPA = 167;     // pulse control - opposite of PWM1
    EPwm2Regs.AQCTLA.bit.CAU = 0b01;
    EPwm2Regs.AQCTLA.bit.CAD = 0b10;
    EPwm2Regs.AQCTLB.bit.CAU = 0b10;
    EPwm2Regs.AQCTLB.bit.CAD = 0b01;

    // Boosterpack Enable
    // pull ENGATE high
    GpioCtrlRegs.GPDGMUX2.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO124 = 1;
    GpioCtrlRegs.GPDPUD.bit.GPIO124 = 0;
    GpioDataRegs.GPDSET.bit.GPIO124 = 1;
    Uint32 j;    // wait 1ms for activation (at 200MHz)
    for (j=0; j<200000; j++) {}

    // QEP Setup
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 0; GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;
    asm(" NOP"); asm(" NOP");
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
 * Handles hardware I/O at 1kHz.
 * - Controls output voltage through pwm
 * - Increments voltage output state machine
 * - Logs motor angular position and desired vout
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
 * QEP Interface
 * - Read QPOSCNT
 * - Convert to an angular position
 *
 * The switch-case body of the code looks a little weird/lopsided because
 * on each interrupt it reads the state, changes the PWM, and then increments
 * to the next state. This is so we are ready to go on the next interrupt.
 *
 * Remember to subtract 1 from the <state> to get the current state when
 * debugging.
 */
interrupt void TimerISR(void)
{
    // logic for switching state
    // switch state every 200 timer0 interrupts (1kHz -> 0.2s)
    if (tmr0_count % 200 == 0) {
        switch (state) {

            // +20V
            // duty cycle of 91.7% to get vo=20V with current parameters
            case 0:
                EPwm1Regs.CMPA.bit.CMPA = 167;
                EPwm2Regs.CMPA.bit.CMPA = 167;
                vo = 20;
                state = 1;  // update state
                break;

            // 0V
            // d=0.5 gives vavg=0
            case 1:
                EPwm1Regs.CMPA.bit.CMPA = 1000;
                EPwm2Regs.CMPA.bit.CMPA = 1000;
                vo = 0;
                state = 2;
                break;

            // -20V
            // mirror of state 0 (same duty cycle, flipped output)
            case 2:
                EPwm1Regs.CMPA.bit.CMPA = 1833;
                EPwm2Regs.CMPA.bit.CMPA = 1833;
                vo = -20;
                state = 3;
                break;

            // 0VDC
            // d=0.5 gives vavg=0
            case 3:
                EPwm1Regs.CMPA.bit.CMPA = 1000;
                EPwm2Regs.CMPA.bit.CMPA = 1000;
                vo = 0;
                state = 0;
                break;
        }
    }

    // QEP reading & logging
    long ticks = EQep1Regs.QPOSCNT;  // convert Uint32 to int32
    float theta = (float) ticks / TICKS_PER_REV;  // TICKS_PER_REV is float
    if (i < LOG_LENGTH) {
        vo_log[i] = vo;
        theta_log[i] = theta;
        i++;
    }

    // Exiting interrupt
    tmr0_count++;
    PieCtrlRegs.PIEACK.all = M_INT1;
}
