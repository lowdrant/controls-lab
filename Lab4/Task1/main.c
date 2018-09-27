/**
 * Lab4 Task1 for ECE4550 Fall 2018
 *
 * Output an analog voltage on J3-10 (ADCINA0)
 * Read said voltage on J3-9 (ADCINA2)
 * v(t) = 1.5 + 0.5cos(2*pi*1000*t)
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-09-27
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


int t = 0;         // time value for voltage waveform
float vo = 2;      // output voltage
float vi;          // ADC instantaneous read value
float vwrit[400];  // desired voltage values
float vread[400];  // stored read voltages
Uint16 i = 0;      // current vi index


interrupt void AdcIsr(void);


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

    // Clocking (fclk = 10MHz)
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 1;   // external source (10 MHz)
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;       // disable PLL to edit
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // clear division register
    ClkCfgRegs.SYSPLLMULT.all = 20;               // clock multiplier (p.334)
    while (ClkCfgRegs.SYSPLLSTS.bit.LOCKS != 1);  // wait for PLL lock
    ClkCfgRegs.SYSCLKDIVSEL.all = 1;              // 1+desired division
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;       // make PLL system clock
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // final clock division

    // DAC Setup (fdac = ftmr, default)
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1;       // turn on DAC clock
    DacaRegs.DACCTL.bit.DACREFSEL = 1;       // use VREFHI
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;      // enable dac
    DacaRegs.DACVALS.all = 4096 * 2.0 / VREF;  // init vout of 2V

    // ADC Setup
    // clocking & triggers: fadc = 50MHz, 12-bit mode
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;    // enable ADC clock
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;    // 200MHz/50Mhz = 4
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;    // power-on ADC
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;  // timer0 trigger
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;    // AIN CHAN0 (pin J3-9)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 39;   // tacq = 200ns (40x SYSCLK)
    // interrupt setup
    // use adc interrupt 1
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 0;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // EOC0 trigger because used SOC0
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // ADCINT1 enable

    // Interrupt Timer (timer0, ftmr)
    // from 200MHz to 100kHz
    CpuTimer0Regs.TCR.bit.TSS = 1;         // stop timer0
    CpuTimer0Regs.PRD.all = 2000 - 1;      // 200Mhz->100kHz
    CpuTimer0Regs.TCR.bit.TRB = 1;         // load timer division
    CpuTimer0Regs.TCR.bit.TIE = 1;         // Timer Interrupt Enable

    // Interrupt Assignment
    // (adca1 - J3-10 is grpa, chose int1)
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
//    PieVectTable.TIMER0_INT = &TimerIsr;  // using timer0
    PieVectTable.ADCA1_INT = &AdcIsr;     // (group a, chose INT1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
//    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
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


/** AdcIsr
 *
 * Handles DAC & ADC control on ADCAINT1
 * In order:
 * Output new vo
 * Read current vi
 * Store vi
 * Compute next vo
 *
 * vo = 1.5 + 0.5 * cos(2*pi*1000*t)
 * Frequency: 100kHz
 *
 */
float f = 2000 * PI * 10e-6;  // 10e-6 from interrupt period
interrupt void AdcIsr(void)
{
    // output vo
    DacaRegs.DACVALS.all = (Uint16) (4096 * vo / VREF);

    // reading vi
    Uint16 tmp = AdcaResultRegs.ADCRESULT0;

    // compute next value
    t += 1;     // increment by interrupt period (100kHz)
    t %= 1000;  // cap at vo period              (1kHz)
    vo = 1.5 + 0.5 * cos(f * t);  // f auto scales t

    // storage
    vi = tmp * VREF / 4096.0;
    if (i < 400) {
        vread[i] = vi;
        vwrit[i] = vo;
        i++;
    }

    // ack
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = M_INT1;
}
