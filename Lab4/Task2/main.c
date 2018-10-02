/**
 * Lab4 Task2 for ECE4550 Fall 2018
 *
 * Implement a notch filter to remove noise from a DAC output
 *
 * Output an analog voltage on J3-10 (ADCINA0)
 * Read said voltage on J3-9 (ADCINA2)
 * v(t) = 1.5 + 0.5cos(2*pi*1000*t) + 0.1*cos(f2*t)
 * Output filtered signal on J7-10
 * Read voltage again on J7-9
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


// noisy signal parameters
int t = 0;          // time value for voltage waveform
float vnoise = 2.1;       // output voltage
float vnoise_r;           // ADC instantaneous read value
float v_j310[400];  // ideal input
float v_j39[400];  // actual input
Uint16 i = 0;       // current vi index

// filtered signal parameters
float v_j710[400];  // ideal output
float v_j79[400];   // actual output

// filter parameters
float alpha = 0.08*PI;
float beta = 0.97;
float a1 = -1.87905;
float a2 = 0.9409;
float b1 = -1.93717;
float b2 = 1;

// signal value trackers
float vfilt = 0;
float vfilt_r = 0;
float vi_d2 = 0;
float vi_d1 = 0;
float vi = 2.1;
float vo_d1 = 0;
float vo_d2 = 0;

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
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1;         // turn on DACA clock
    CpuSysRegs.PCLKCR16.bit.DAC_B = 1;         // turn on DACB clock
    DacaRegs.DACCTL.bit.DACREFSEL = 1;         // use VREFHI
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;        // enable DAVA
    DacaRegs.DACVALS.all = 4096 * 2.1 / VREF;  // init voutA of 2.1V
    DacbRegs.DACCTL.bit.DACREFSEL = 1;         // use VREFHI
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;        // enable DACB
    DacbRegs.DACVALS.all = 4096 * 0 / VREF;    // init voutB of 0V

    // ADC Setup | clocking & triggers: fadc = 50MHz, 12-bit mode
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;    // enable ADC clock
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;    // 200MHz/50Mhz = 4
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;    // power-on ADC
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;  // timer0 trigger
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;    // AIN CHAN2 (pin J3-9)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 39;   // tacq = 200ns (40x SYSCLK)
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 1;  // timer0 trigger
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;    // AIN CHAN4 (pin J7-9)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 39;   // tacq = 200ns (40x SYSCLK)
    // interrupt setup | GRPA_INT1
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 0;
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // EOC0 trigger because used SOC0
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // ADCINT1 enable


    // Interrupt Timer (timer0, ftmr)
    // from 200MHz to 100kHz
    CpuTimer0Regs.TCR.bit.TSS = 1;     // stop timer0
    CpuTimer0Regs.PRD.all = 2000 - 1;  // 200Mhz->100kHz
    CpuTimer0Regs.TCR.bit.TRB = 1;     // load timer division
    CpuTimer0Regs.TCR.bit.TIE = 1;     // Timer Interrupt Enable

    // Interrupt Assignment
    // (adca1 - J3-10 is grpa, chose int1 arbitrarily)
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.ADCA1_INT = &AdcIsr;  // (group a, INT1)
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    IER = 1;

    // wait 500us at SYSCLK = 200MHz
    // (1000 cycles)
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
 * Outputs & reads noisy signal
 * Filters noisy signal
 * Outputs & reads filtered signal
 *
 * vo = 1.5 + 0.5 * cos(2*pi*1000*t) + 0.1*cos(f2*t)
 * Frequency: 100kHz
 *
 */
float f1 = 2000 * PI * 10e-6;  // 10e-6 from interrupt period
float f2 = 8000 * PI * 10e-6;  // 10e-6 from interrupt period
interrupt void AdcIsr(void)
{
    // output voltages
    DacaRegs.DACVALS.all = (Uint16) (4096 * vnoise / VREF);
    DacbRegs.DACVALS.all = (Uint16) (4096 * vfilt / VREF);

    // reading input voltages
    Uint16 tmpj39 = AdcaResultRegs.ADCRESULT0;
    Uint16 tmpj79 = AdcaResultRegs.ADCRESULT1;

    // compute next values
    t += 1;     // increment by interrupt period (100kHz)
    t %= 1000;  // cap at vo period              (1kHz)
    vfilt  = vnoise + b1*vi_d1 + b2*vi_d2 - a1*vo_d1 - a2*vo_d2;
    vi_d2 = vi_d1;  // filter values after computation, but before change
    vi_d1 = vnoise;
    vo_d2 = vo_d1;
    vo_d1 = vfilt;
    vnoise = 1.5 + 0.5*cos(f1*t) + 0.1*cos(f2*t);  // f auto scales t


    // storage
    vnoise_r = tmpj39 * VREF / 4096.0;
    vfilt_r = tmpj79 * VREF / 4096.0;
    if (i < 400) {
        v_j310[i] = vnoise;
        v_j39[i] = vnoise_r;
        v_j710[i] = vfilt;
        v_j79[i] = vfilt_r;
        i++;
    }

    // ack
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = M_INT1;
}
