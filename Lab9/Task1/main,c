/**
 * Lab9 Task2 for ECE4550 Fall 2018
 *
 * Read IMU data over I2C interface
 * fclk = 200 MHz (req)
 * fbus = 50 kHz  (req)
 * ftmr = 2 Hz    (req)
 * fi2c = 10 MHz  (semi-arb)
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-11-15
 */
#include "F2837xD_device.h"
#include "math.h"
#ifndef IMU_ADDR
#define IMU_ADDR 0x69
#endif

interrupt void TimerISR(void);  // timer0-based interrupt for lab i/o
char request_byte(char, char);
char xlobit, xhibit, ylobit, yhibit, zlobit, zhibit;
float32 xacc = 0, yacc = 0, zacc = 0;


int main(void)
{
    //==============================Begin Setup==============================
    DINT; EALLOW; WdRegs.WDCR.all = 0x68;

    // Clocking (fclk = 200 MHz)
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 1;   // external source (10 MHz)
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;       // disable PLL to edit
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // clear division regist-er
    ClkCfgRegs.SYSPLLMULT.all = 20;               // clock multiplier (p.334)
    while (ClkCfgRegs.SYSPLLSTS.bit.LOCKS != 1);  // wait for PLL lock
    ClkCfgRegs.SYSCLKDIVSEL.all = 1;              // 1+desired division
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;       // make PLL system clock
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // final clock division

    // Canary LED
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;
    GpioDataRegs.GPASET.bit.GPIO31 = 1;  // turn off led so know code works

    // I2C GPIO Setup (Module A)
    // pull-up SDA & SCL, both open-drain mode
    GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;  // SDAA (pull-up)
    GpioCtrlRegs.GPDMUX1.bit.GPIO104 = 1;
    GpioCtrlRegs.GPDPUD.bit.GPIO104 = 0;
    GpioCtrlRegs.GPDODR.bit.GPIO104 = 1;    // open drain mode
    GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;  // SCLA (pull-up)
    GpioCtrlRegs.GPDMUX1.bit.GPIO105 = 1;
    GpioCtrlRegs.GPDPUD.bit.GPIO105 = 0;
    GpioCtrlRegs.GPDODR.bit.GPIO105 = 1;    // open drain mode

    // I2C Communication Setup (Module A)
    // master-rx, master-tx
    //
    // Choose I2C module frequency:
    // I2C freq limited to [7Mhz, 12Mhz] by protocol
    // I2C freq = sysclkfreq / (IPSC+1)
    // Choose I2C_f = 10Mhz
    // => IPSC = 20-1 = 19
    //
    // Choose SCL frequency:
    // (p. 2203 in TRM)
    // bus freq = 50 kHz, specified by lab
    // bus period = time_low + time_high
    //            = Tmod*(I2CCLKL+d) + Tmod*(I2CCLKH+d)
    //            = 0.00002s = 1 / 50kHz
    //
    // Tmod = 1 / I2C_f
    // d = 5, IPSC>1
    //
    // choose time_high = time_low for symmetric waveform
    // => I2CCLKL = I2CCLKH
    // => (I2CCLKL+d) / I2C_f = 0.00001s
    // => I2CCLKL = 95
    //
    CpuSysRegs.PCLKCR9.bit.I2C_A = 1;        // clk en
    asm(" NOP"); asm(" NOP");
    I2caRegs.I2CMDR.bit.IRS = 0;             // reset mode
    I2caRegs.I2CPSC.bit.IPSC = 20 - 1;       // i2c clkprediv
    I2caRegs.I2CCLKL = 95;                   // bus clkdiv hi-time
    I2caRegs.I2CCLKH = 95;                   // bus clkdiv lo-time
    I2caRegs.I2CMDR.bit.IRS = 1;             // exit reset mode

    // Timer0 Interrupt (ftmr=5kHz)
    CpuTimer0Regs.TCR.bit.TSS = 1;      // stop timer0
    CpuTimer0Regs.PRD.all = 10e6 - 1;   // 200Mhz->2Hz
    CpuTimer0Regs.TCR.bit.TRB = 1;      // load timer division
    CpuTimer0Regs.TCR.bit.TIE = 1;      // Timer Interrupt Enable
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.TIMER0_INT = &TimerISR;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER = 1;
    CpuTimer0Regs.TCR.bit.TSS = 0;      // Re-enable timer0

    // Power on Booster Pack
    while (I2caRegs.I2CMDR.bit.STP == 1);
    I2caRegs.I2CCNT = 2;
    I2caRegs.I2CSAR.bit.SAR = 0x69;         // specify bmi160
    I2caRegs.I2CDXR.bit.DATA = 0x7E;        // write to cmd register
    I2caRegs.I2CMDR.all = 0x2E20;
    while (I2caRegs.I2CSTR.bit.XRDY == 0);  // send `on` to cmd
    I2caRegs.I2CDXR.bit.DATA = 0x11;

//    WdRegs.WDCR.all = 0x28;
    EDIS; EINT;
    //===============================End Setup===============================

    // feed the dog
    while (1) {
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}


/**
 *
 * I2C Communication @ 2Hz
 *
 */
interrupt void TimerISR(void)
{
    // get xdata
    while (I2caRegs.I2CSTR.bit.XRDY == 0);   // wait for clear
    I2caRegs.I2CSAR.bit.SAR = 0x69;
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.bit.DATA = 0x14;          // send register
    I2caRegs.I2CMDR.all = 0x2E20;
    while(I2caRegs.I2CSTR.bit.ARDY == 0);    // wait to read
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2C20;
    while(I2caRegs.I2CSTR.bit.RRDY == 0);
    xlobit = I2caRegs.I2CDRR.bit.DATA;

    while (I2caRegs.I2CSTR.bit.XRDY == 0);   // wait for clear
    I2caRegs.I2CSAR.bit.SAR = 0x69;
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.bit.DATA = 0x15;          // send register
    I2caRegs.I2CMDR.all = 0x2E20;
    while(I2caRegs.I2CSTR.bit.ARDY == 0);    // wait to read
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2C20;
    while(I2caRegs.I2CSTR.bit.RRDY == 0);
    xhibit = I2caRegs.I2CDRR.bit.DATA;

    xacc = (xhibit << 8) + xlobit;

//    xlobit = request_byte(IMU_ADDR, 0x12);
//    xhibit = request_byte(IMU_ADDR, 0x13);

    // get ydata
//    ylobit = request_byte(IMU_ADDR, 0x14);
//    yhibit = request_byte(IMU_ADDR, 0x15);
//
//    // get zdata
//    zlobit = request_byte(IMU_ADDR, 0x16);
//    zhibit = request_byte(IMU_ADDR, 0x17);

    xacc = (xhibit << 8) + xlobit;
//    yacc = (yhibit << 8) + ylobit;
//    zacc = (zhibit << 8) + zlobit;

    PieCtrlRegs.PIEACK.all = M_INT1;
}


char request_byte(char addr, char reg)
{
    while (I2caRegs.I2CSTR.bit.XRDY == 0);   // wait for clear
    I2caRegs.I2CSAR.bit.SAR = addr;
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.bit.DATA = reg;          // send register
    I2caRegs.I2CMDR.all = 0x2620;
    while(I2caRegs.I2CSTR.bit.ARDY == 0);    // wait to read
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2420;
    while(I2caRegs.I2CSTR.bit.RRDY == 0);
    char out = I2caRegs.I2CDRR.bit.DATA;

    return out;
}
