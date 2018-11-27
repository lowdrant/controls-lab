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
//============================Begin I2C Constants=============================
#ifndef IMU_ADDR
#define IMU_ADDR 0x69            // i2c address of BMI160 IMU chip
#endif
#ifndef CMD_REGISTER
#define CMD_REGISTER 0x7E        // controls power/speed status (p.81)
#endif
#ifndef ACC_RANGE_REGISTER
#define ACC_RANGE_REGISTER 0x41  // stores max true acceleration val (p.56)
#endif
#ifndef X_ACC_LOW_BYTE
#define X_ACC_LOW_BYTE 0x12      // lowest register that stores acceleration
#endif
#ifndef MAX_ACC_BINARY
#define MAX_ACC_BINARY 0xFFFF    // max raw acceleration value - 2 bytes
#endif
//=============================End I2C Constants==============================

// I2C Handlers
interrupt void TimerISR(void);  // timer0-based interrupt for lab i/o
void delay2us(void);            // 2 microsecond delay between writes
Uint16 acc_max = 0;             // max abs reading of acceleration (default 2g)
float32 acc_scale = 0;          // max reading/ max 0bVal for conversion
char curbyte = 0;               // generic intermediate data byte holder

// Data Loggers
Uint32 i = 0;
char acc_arr[6];
float32 xacc = 0, yacc = 0, zacc = 0;


int main(void)
{
    //============================Begin MCU Setup=============================
    DINT; EALLOW; WdRegs.WDCR.all = 0x68;

    // Clocking (fclk = 200 MHz)
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 1;   // external source (10 MHz)
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;       // disable PLL to edit
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // clear division register
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
    GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;  // SDAA (pulled-up)
    GpioCtrlRegs.GPDMUX1.bit.GPIO104 = 1;
    GpioCtrlRegs.GPDPUD.bit.GPIO104 = 0;
    GpioCtrlRegs.GPDODR.bit.GPIO104 = 1;    // open drain mode
    GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;  // SCLA (pulled-up)
    GpioCtrlRegs.GPDMUX1.bit.GPIO105 = 1;
    GpioCtrlRegs.GPDPUD.bit.GPIO105 = 0;
    GpioCtrlRegs.GPDODR.bit.GPIO105 = 1;

    // I2C Communication Setup (Module A)
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

    // Timer0 Interrupt
    // Ttmr = 0.5s -> ftmr = 2Hz
    CpuTimer0Regs.TCR.bit.TSS = 1;
    CpuTimer0Regs.PRD.all = 10e6 - 1;   // 200Mhz->2Hz
    CpuTimer0Regs.TCR.bit.TRB = 1;
    CpuTimer0Regs.TCR.bit.TIE = 1;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.TIMER0_INT = &TimerISR;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER = 1;
    CpuTimer0Regs.TCR.bit.TSS = 0;
    //=============================End MCU Setup==============================

    //============================Begin IMU Setup=============================
    // Power up
    while (I2caRegs.I2CMDR.bit.STP == 1);  // seize the means of communication
    I2caRegs.I2CCNT = 2;
    I2caRegs.I2CSAR.bit.SAR = IMU_ADDR;
    I2caRegs.I2CDXR.bit.DATA = CMD_REGISTER;
    I2caRegs.I2CMDR.all = 0x2620;
    delay2us();
    while (I2caRegs.I2CSTR.bit.XRDY == 0);
    I2caRegs.I2CDXR.bit.DATA = 0x11;       // send `on` to cmd
    delay2us();

    // Compute acceleration resolution
    while (I2caRegs.I2CSTR.bit.XRDY == 0);
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.bit.DATA = ACC_RANGE_REGISTER;       // get max g
    I2caRegs.I2CMDR.all = 0x2620;
    delay2us();
    while(I2caRegs.I2CSTR.bit.ARDY == 0);
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2C20;
    while(I2caRegs.I2CSTR.bit.RRDY == 0);
    curbyte = I2caRegs.I2CDRR.bit.DATA;
    switch (curbyte) {                                   // p.56-57 datasheet
    case 0b0101:
        acc_max = 4;
        break;
    case 0b1000:
        acc_max = 8;
        break;
    case 0b1100:
        acc_max = 16;
        break;
    default:
        acc_max = 2;                                     // default/most common
        break;
    }
    acc_scale = (float32) 2 * acc_max / MAX_ACC_BINARY;  // compute g per raw
    //=============================End IMU Setup==============================

    // feed the dog
    WdRegs.WDCR.all = 0x28; EDIS; EINT;
    while (1) {
        feed();
    }
}


/** TimerISR
 *
 * Read x,y,z acceleration using I2C
 * Frequency: 2 Hz
 *
 * Only writes the lowest register to get data from (X_ACC_LOW_BYTE).
 * The BMI160 auto-increments the register from which it sends data with
 * successive repeated-start reads.
 *
 */
interrupt void TimerISR(void)
{
    while (I2caRegs.I2CMDR.bit.STP == 1);  // seize the means of communication
    I2caRegs.I2CSAR.bit.SAR = IMU_ADDR;

    // request lowest data register
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.bit.DATA = X_ACC_LOW_BYTE;
    I2caRegs.I2CMDR.all = 0x2620;          // DO NOT RELEASE

    // read data in one go
    //   6 bytes - x,y,z; 2 bytes each
    //   p.46 for order
    while(I2caRegs.I2CSTR.bit.ARDY == 0);
    I2caRegs.I2CCNT = 6;
    I2caRegs.I2CMDR.all = 0x2C20;          // release
    for (i = 0; i < 6; i++) {
        while(I2caRegs.I2CSTR.bit.RRDY == 0);
        acc_arr[i] = I2caRegs.I2CDRR.bit.DATA;
        feed();
    }

    xacc = ((acc_arr[1] << 8) + acc_arr[0]) * acc_scale;
    yacc = ((acc_arr[3] << 8) + acc_arr[2]) * acc_scale;
    zacc = ((acc_arr[5] << 8) + acc_arr[3]) * acc_scale;

    PieCtrlRegs.PIEACK.all = M_INT1;
}


/** delay2us
 *
 * Provides 2 microsecond delay @200MHz for I2C interface
 *
 * From p.90 of the data sheet, the BMI160 chip requires a 2 us delay
 * following a write command while in normal mode.
 *
 */
void delay2us(void)
{
    Uint16 i;
    for (i = 0; i < 400; i++);
}


/** feed
 *
 * Feeds watchdog timer
 *
 */
void feed(void)
{
    WdRegs.WDKEY.all = 0x55;
    WdRegs.WDKEY.all = 0xAA;
}
