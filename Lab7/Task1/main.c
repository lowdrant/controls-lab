/**
 * Lab7 Task1 for ECE4550 Fall 2018
 *
 * Goal:
 * Generate a reference trajectory for
 * motion exceeding motor physical limits
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-11-01
 */
#include "F2837xD_device.h"
#include "math.h"


//========================== Begin System Constants ==========================
// Universal Constants
#ifndef PI
#define PI 3.14159
#endif
float32 TWOPI = 2*PI;

// Hardware Limitations
#ifndef VIN
#define VIN 24              // System Input Voltage
#endif
#ifndef TICKS_PER_REV
#define TICKS_PER_REV 1000  // encoder ticks per 2pi
#endif
#ifndef A
#define A 98.8363           // Motor params (Inertia)
#endif
#ifndef B
#define B 559.8633
#endif

// User-Controllable
#ifndef PWM_MAX_COUNT
#define PWM_MAX_COUNT 2000  // PWM Maximum Count
#endif
#ifndef LOG_LENGTH
#define LOG_LENGTH 2000     // Number of points in data loggers
#endif
#ifndef T
#define T 0.001             // Sampling Period (1kHz)
#endif
#ifndef COUNT_PER_SEC
#define COUNT_PER_SEC 1000  // 1/T
#endif
#ifndef LAMBDA_R
#define LAMBDA_R 50         // Reg/Est poles
#endif
#ifndef LAMBDA_E
#define LAMBDA_E 200
#endif

// Trajectory Constants
float32 P1 = 0;       // Desired positions
float32 P2 = 10*PI;
float32 SC = 100;     // Speed limit
float32 AC = 4000;    // Acceleration limit
float32 ta = 0.0250;  // SC/AC (compiler doesn't like math in globals)
float32 ts = 0.2892;  // (P2-P1)/SC - SC/AC

//=========================== End System Constants ===========================


// Controller Setup
interrupt void TimerISR(void);  // timer0-based interrupt for lab i/o
Uint16 pwmCMP(float);           // pwm compare value calculator
Uint32 ISR_count = 0;           // count interrupts (generally useful)

// Trajectory Parameters
// to be set in TimerISR (compiler doesn't like them set out here)
float32 pos_start;  // starting position of traj
float32 pos_end;    // ending position of traj
float32 t1;         // initial trajectory time (not const b/c traj changes)
float32 t2;         // ending trajectory time (not const b/c traj changes)
float32 dir = 1;    // sign tracker for trajectory math

// Data Loggers
float32 yref_log[LOG_LENGTH];      // desired motor pos
float32 yrefdot_log[LOG_LENGTH];   // desired motor vel
float32 yref2dot_log[LOG_LENGTH];  // desired motor accel

float32 time = 0;
float32 yref, yrefdot, yref2dot;


int main(void)
{
    // Initialize "Constants"
    // declared globally
    // defined here to avoid compiler getting mad at math expressions
    ta = SC / AC;
    ts = (P2 - P1)/SC - SC/AC;
    t1 = 0;
    t2 = 2*ta + ts;
    dir = 1;

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
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;      // fpwm = fclk,pwm/(2*TBPRD*CLKDIV*HSPCLKDIV)
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.AQCTLA.bit.CAU = 0b10;     // 1A high on upcount
    EPwm1Regs.AQCTLA.bit.CAD = 0b01;     // 1A low on downcount
    EPwm1Regs.AQCTLB.bit.CAU = 0b01;     // 1B low on upcount
    EPwm1Regs.AQCTLB.bit.CAD = 0b10;     // 1B high in downcount

    // PWM2 Setup: Booster Pack Phase B Control
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2=0; GpioCtrlRegs.GPAMUX1.bit.GPIO2=1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3=0; GpioCtrlRegs.GPAMUX1.bit.GPIO3=1;
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
    // pull engate LOW for testing
    GpioCtrlRegs.GPDGMUX2.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO124 = 1;
    GpioCtrlRegs.GPDPUD.bit.GPIO124 = 0;
    GpioDataRegs.GPDCLEAR.bit.GPIO124 = 1;
    Uint32 j;    // wait 1ms for deactivation (at 200MHz)
    for (j=0; j < 200000; j++) {}

    // QEP Setup
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20=0; GpioCtrlRegs.GPAMUX2.bit.GPIO20=1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21=0; GpioCtrlRegs.GPAMUX2.bit.GPIO21=1;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23=0; GpioCtrlRegs.GPAMUX2.bit.GPIO23=1;
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;  // clock
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
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // enable timer group interrupt
    IER = 1;                            // enable group 1 interrupts

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // system clock to pwm
    CpuTimer0Regs.TCR.bit.TSS = 0; WdRegs.WDCR.all = 0x28; EDIS; EINT;
    //===============================End Setup===============================

    // Feed watchdog
    while (1) {
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}


/** TimerISR
 *
 * Handles control at 1kHz
 *
 * Generates a reference trajectory for the motor as specified in 7.2.2.1.
 *
 */
interrupt void TimerISR(void)
{
    // Trajectory Initialization
    // change direction every 0.5s
    if (ISR_count % (Uint32) ((float32) COUNT_PER_SEC / 2) == 0) {
        pos_start = (pos_start == P1) ? P2 : P1;  // swap start & end pos
        pos_end = (pos_end == P1) ? P2 : P1;
        dir *= -1;  // switch signs for motion calcs
        t1 = time;  // move time parameters
        t2 = 2*ta + t2 + t1;
    }
    // ensure start at P1, going positive
    // necessary b/c 0 mod x == 0 (if statement above)
    if (time == 0) {
        pos_start = P1;
        pos_end = P2;
        dir = 1;
    }

    // Trajectory Computation
    // dir tracks the +/- sign for CW vs. CCW movement
    if (time < t1) {              // Rest @start
        yref = pos_start;
        yrefdot = 0;
        yref2dot = 0;
    } else if (time < t1 + ta) {  // Speeding up
        yref = pos_start + dir*0.5*AC*pow(time - t1, 2);
        yrefdot = dir * AC * (time - t1);
        yref2dot = dir * AC;
    } else if (time < t2 - ta) {  // Coasting
        yref = 0.5*(pos_start+pos_end) + dir*SC*(time - 0.5*(t1+t2));
        yrefdot = dir * SC;
        yref2dot = 0;
    } else if (time < t2) {       // Slowing down
        yref = pos_end - dir*0.5*AC*pow(t2 - time, 2);
        yrefdot = dir * AC * (t2 - time);
        yref2dot = -dir * AC;
    } else if (time > t2) {
        yref = pos_end;
        yrefdot = 0;
        yref2dot = 0;
    }

    // Data Logging
    if (ISR_count < LOG_LENGTH) {
        yref_log[ISR_count] = yref;          // pos
        yrefdot_log[ISR_count] = yrefdot;    // vel
        yref2dot_log[ISR_count] = yref2dot;  // accel
    }

    // Exiting Interrupt
    ISR_count++;
    time = ISR_count * T;  // num_interrupts * time_per_interrupt
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
Uint16 pwmCMP(float32 vo)
{
    float32 d = (vo / VIN + 1) / 2;       // duty cycle
    return round((1-d) * PWM_MAX_COUNT);  // convert to a counter value
}
