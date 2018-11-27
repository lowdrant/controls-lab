/**
 * Lab9 Task1 for ECE4550 Fall 2018
 *
 * I2C interface to IMU for setup
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

interrupt void TimerISR(void);  // timer0-based interrupt for lab i/o
void delay2us(void);            // 2 microsecond delay between writes
Uint16 chip_id = 0x00ff;        // i2c imu address initial guess
Uint16 pmu_status = 0xffff;     // pmu status var
char curbyte = 0;               // byte from over i2c
Uint32 i = 0;                   // iteration var
float32 data = 0;


int main(void)
{
    //==============================Begin Setup==============================
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

    //==========================Talk to Booster Pack=========================
    EDIS;  // re-enable register protections (for defensive programming)

    /** Power-up BMI160 (0x69)
     *  Tell BMI160 to prepare to have CMD register written
     *   - Write cmd register address (0x7E) to BMI160
     *   - Write value to be stored (0x11) in cmd to BMI16
     * tx 2 bytes
     */
    while (I2caRegs.I2CMDR.bit.STP == 1);   // wait for bus to free up
    I2caRegs.I2CSAR.bit.SAR = 0x69;         // specify BMI160 address
    I2caRegs.I2CCNT = 2;                    // write 2 bytes
    I2caRegs.I2CDXR.bit.DATA = 0x7E;        // send register to be written
    I2caRegs.I2CMDR.all = 0x2620;           // master tx, no stop cond
    delay2us();                             // wait after command
    while (I2caRegs.I2CSTR.bit.XRDY == 0);  // wait for 1st byte sent
    I2caRegs.I2CDXR.bit.DATA = 0x11;        // send `turn on power` val
    delay2us();

    /** Verify BMI160 power (check PMU_Status)
     *  Tell BMI160 to send value stored in pmu_status register
     *   - Write PMU_Status address (0x03)
     *   - Read value
     * tx 1 byte
     * rx 1 byte
     *
     * This isn't specifically required by the lab check off, but being able
     * to do so is important for requesting data.
     *
     * This must be done separately from the cmd write above to allow the
     * change to take effect.
     */
    for (i = 0; i < 760e3; i++);  // wait 4ms for changes to take effect
    while (I2caRegs.I2CSTR.bit.XRDY == 0);  // tx
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.bit.DATA = 0x03;  // pmu_status addr
    I2caRegs.I2CMDR.all = 0x2620;     // tx, no stop
    delay2us();
    while(I2caRegs.I2CSTR.bit.ARDY == 0);  // rx
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2420;     // rx, no stop
    while(I2caRegs.I2CSTR.bit.RRDY == 0);
    pmu_status = I2caRegs.I2CDRR.bit.DATA;

    /** Request value stored in chip_id register
     *  Tell BMI160 to send value in chip_id register
     *   - Write chip_id address (0x00)
     *   - Read value
     * tx 1 byte
     * rx 1 byte
     */
    // tx: request chip_id
    while (I2caRegs.I2CSTR.bit.XRDY == 0);  // wait for clear
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.bit.DATA = 0x00;
    I2caRegs.I2CMDR.all = 0x2620;           // tx, no stop
    delay2us();
    // rx requested value
    while(I2caRegs.I2CSTR.bit.ARDY == 0);   // wait to switch to rx
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x2C20;           // rx, yes stop cond
    while(I2caRegs.I2CSTR.bit.RRDY == 0);   // wait for read-readiness
    chip_id = I2caRegs.I2CDRR.bit.DATA;
    //==========================Talk to Booster Pack=========================

    EALLOW;
    WdRegs.WDCR.all = 0x28; EDIS; EINT;
    EDIS; EINT;
    //===============================End Setup===============================

    // feed the dog
    while (1) {
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
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
    for (i = 0; i < 100; i++);
}

/** TimerISR
 *
 * I2C Communication @ 2Hz
 *
 */
interrupt void TimerISR(void)
{

}
