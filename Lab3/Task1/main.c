/**
 * Lab3 Task1 for ECE4550 Fall 2018
 *
 * Generate a 1kHz squarewave on J8-10 (GPIO6)
 * Use oscillator source XTAL
 * fclk = 20MHz
 *
 * 20MHz to 1kHz => flip gpio every 20000 clock cycles
 *
 * Authors: Marion Anderson, Aditya Retnanto
 * Date: 2018-09-13
 */
#include "F2837xD_device.h"
#include "F2837xD_pievect.h"


interrupt void squareISR(void);
Uint32 loop_count = 0;

int main(void)
{
    DINT;  // disable interrupts for safety
    EALLOW;
    WdRegs.WDCR.all = 0x68;

    // gpio6 as output
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;  // start low

    // enable blue led for debugging purposes
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;
    GpioDataRegs.GPASET.bit.GPIO31 = 1;  // turn off led so know code works

    // oscillator clock (fosc)
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 1;   // external source

    // system clock (fclk) as PLL
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;       // disable PLL to edit
    ClkCfgRegs.SYSCLKDIVSEL.all = 0;              // clear division register
    ClkCfgRegs.SYSPLLMULT.all = 20;                // clock divider (p.334)
    while (ClkCfgRegs.SYSPLLSTS.bit.LOCKS != 1);  // wait for PLL lock
    ClkCfgRegs.SYSCLKDIVSEL.all = 11;              // 1+desired division
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;       // make PLL system clock
    ClkCfgRegs.SYSCLKDIVSEL.all = 10;              // final clock division

    // interrupt clock (timer0, ftmr)
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 0;  // turn off timer0 before edit
    CpuTimer0Regs.TCR.bit.TSS = 1;         // stop timer0
    CpuTimer0Regs.PRD.all = 20000 + 1;      // 20k fclk per ftmr (20Mhz->1kHz)
    CpuTimer0Regs.TCR.bit.TRB = 1;         // load timer division
    CpuTimer0Regs.TCR.bit.TIE = 1;         // Timer Interrupt Enable
    CpuTimer0Regs.TCR.bit.TSS = 0;         // restart timer0
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;

    WdRegs.WDCR.all = 0x28;
    EDIS;

    /* gpio6 interrupt setup
     *
     * Using TIMER0 for the square wave
     * TIMER0 => INT1.y, INTx.7 => INT1.7
     * The interrupt is in Group 1, Interrupt 7
     *
     * 1. Disable interrupts in the CPU
     * 2. Enable peripheral interrupts
     * 3. Assign interrupt functions to the peripheral interrupt table
     * 4. Enable the appropriate peripheral interrupt
     * 5. Enable interrupt path for group 1
     * 6. Re-enable interrupts in the CPU
     */
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;     // enable interrupts
    PieVectTable.TIMER0_INT = &squareISR;  // assign square wave to TIMER0
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;     // enable TIMER0 interrupt
    IER = 1;                               // enable interrupt path for Group 1
    EINT; // reenable interrupts

    while (1) {
        // flash LED to show code is running
        if (loop_count % 10000 == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
        }
        loop_count++;

        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}


/** squareISR
 * Square wave interrupt on GPIO6
 *
 * global vars also set here
 */
interrupt void squareISR(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO6 = 1;  // toggle voltage on pin 6
    PieCtrlRegs.PIEACK.all = M_INT1;       // acknowledge group1 interrupt to clear
}
