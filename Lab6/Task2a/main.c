/**
 * Lab6 Task2a for ECE4550 Fall 2018
 *
 * Goal:
 * Move a DC motor between 2 set angular positions using
 * state space PI control. Transition time should take 1s.
 *
 * Motor model based off of data collected in Lab5 Task2
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-10-18
 */
#include "F2837xD_device.h"
#include "math.h"

#ifndef VIN            // System Input Voltage
#define VIN 24
#endif
#ifndef PWM_MAX_COUNT  // PWM Maximum Count
#define PWM_MAX_COUNT 2000
#endif
#ifndef PI
#define PI 3.14159
#endif
#ifndef TWOPI          // to reduce computation time
#define TWOPI 2*PI
#endif
#ifndef TICKS_PER_REV  // encoder ticks per 2pi
#define TICKS_PER_REV 1000
#endif
#ifndef LOG_LENGTH     // num data points in data loggers
#define LOG_LENGTH 2000
#endif
#ifndef T              // sampling period from f=1kHz
#define T 0.001
#endif
#ifndef A              // motor control parameters
#define A 98.2499
#endif
#ifndef B
#define B 546.9961
#endif
#ifndef LAMBDA_R
#define LAMBDA_R 1
#endif
#ifndef P1             // the two motor positions to jump between
#define P1 0
#endif
#ifndef P2
#define P2 2*PI
#endif

// Control Gains
float K11 = 1/B * 3 * LAMBDA_R * LAMBDA_R;
float K12 = 1/B * (3*LAMBDA_R - A);
float K2 = 1/B * 3 * LAMBDA_R * LAMBDA_R * LAMBDA_R;

// Controller Variables
float r = P1;    // desired position
float sigma[2];  // regulator state variable
float u;         // applied voltage
float y;         // current position
float x1hat[2];  // motor position
float x2hat[2];  // motor velocity

// Motor Controllers
interrupt void TimerISR(void);  // timer0-based interrupt for lab i/o
Uint16 pwmCMP(float);           // pwm compare value calculator
Uint16 tmr0_counter = 0;        // tracks timer interrupt entries for state switching

// Data Loggers
Uint16 i;                 // index of data arrays
float u_log[LOG_LENGTH];  // log desired average voltage
float y_log[LOG_LENGTH];  // log actual angular position
long ticks;
float theta;


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
    EPwm1Regs.TBPRD = PWM_MAX_COUNT;     // vres = 24mV = 2V/TBPRD = 48/2000
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;   // no further clock division needed
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;      // fpwm = fclk,pwm / (2*TBPRD*CLKDIV*HSPCLKDIV)
    EPwm1Regs.CMPA.bit.CMPA = 0;
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
    EPwm2Regs.CMPA.bit.CMPA = 0;
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
 * Handles motor control at 1kHz
 *
 * State space PI position controller with transition time of 1 second
 *
 */
interrupt void TimerISR(void)
{
    // Sense (position)
    ticks = EQep1Regs.QPOSCNT;  // convert Uint32 to int32
    y = (float) ticks * TWOPI / TICKS_PER_REV;

    // Control Model
    // output voltage
    u = -K11*x1hat[1] - K12*x2hat[1] - K2*sigma[1];
    #ifdef ANTIWINDUP
    if (u > VIN || u < -VIN) {
        u = -K11*xhat1[0] - K12*x2hat[0] - K2*sigma[0];
        x1hat[0] = x1hat[1];  // make current, previous (AFTER antiwindup)
        x2hat[0] = x2hat[1];
        sigma[0] = sigma[1];
    } else {  // If saturated, don't increment state vars
        x1hat[1] = x1hat[1] + T * x2hat[0];
        x2hat[1] = x2hat[0] + T * ((-A-B)*x2hat[0] + B*u);  // TODO: check u order
        sigma[1] = sigma[1] + T * (y - r);
    }
    #else
    x1hat[0] = x1hat[1];  // make current, previous (AFTER antiwindup)
    x2hat[0] = x2hat[1];
    sigma[0] = sigma[1];
    x1hat[1] = x1hat[0] + T * x2hat[0];
    x2hat[1] = x2hat[0] + T * ((-A-B)*x2hat[0] + B*u);  // TODO: check u order
    sigma[1] = sigma[0] + T * (y - r);
    #endif

    // Actuate (motor voltage)
    EPwm1Regs.CMPA.bit.CMPA = pwmCMP(u);
    EPwm2Regs.CMPA.bit.CMPA = pwmCMP(u);

    // Data Logging
    if (i < LOG_LENGTH) {
        u_log[i] = u;
        y_log[i] = y;
        i++;
    }

    // Exiting interrupt
    tmr0_counter++;
    // jump between positions every half-log (want 2 cycles' data)
    if (tmr0_counter % LOG_LENGTH/2 == 0) {
        if (r == P1) r = P2;
        if (r == P2) r = P1;
        tmr0_counter = 0;
    }
    PieCtrlRegs.PIEACK.all = M_INT1;
}


/** pwmCMP
 *
 * Computes PWM counter compare value from desired output voltage,
 * assuming that the output is composed exclusively of +/-Vin DC pulses.
 *
 * Notes:
 * - Duty Cycle = vo/2VIN + 0.5 (from the fact that output is +/- DC pulses)
 * - Duty cycle inversely corresponds to counter compare value. This is
 *   because an 80% duty cycle requires that the PWM trigger at 20% of the max
 *   count to cover 80% of the counter cycle.
 * - Uint16 because the PWM time-base counter is 16-bit (p.1827 of TRM)
 */
Uint16 pwmCMP(float vo)
{
    float d = (vo / VIN + 1) / 2;         // duty cycle
    return round((1-d) * PWM_MAX_COUNT);  // convert to a counter value
}
