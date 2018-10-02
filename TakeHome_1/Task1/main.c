/**
 * Take-Home Lab1 (Week 5) for ECE4550 Fall 2018
 *
 * Create a state machine with 4-states, each controlling a different LED in a different way
 *
 * Authors: Marion Anderson
 * Date: 2018-09-28
 */

#include "F2802x_Device.h"

interrupt void pressISR(void);   // detects button press (State Changer)
interrupt void blinkISR(void);   // flashes leds (State Executor)
volatile Uint16 loop_count = 0;  // software clock divider for different states
volatile Uint16 s = 0;           // state machine state tracker
volatile Uint16 btstate;         // button state (for debugging)


int main(void)
{
    // ==============================Begin Setup==============================
    DINT; EALLOW; SysCtrlRegs.SCSR = 0b10; SysCtrlRegs.WDCR = 0x68;

    // led outputs
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;
    GpioDataRegs.GPASET.bit.GPIO0 = 1;  // force all leds off
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    GpioDataRegs.GPASET.bit.GPIO2 = 1;
    GpioDataRegs.GPASET.bit.GPIO3 = 1;

    // pushbutton input on gpio12
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 0b01;   // 3-sample qualification
    GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0x0EF;  // no Type I errors at this rate

    // system clock setup (10MHz ref -> 50MHz)
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;       // PLL bypass
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;      // ignore failures for now
    SysCtrlRegs.PLLCR.bit.DIV = 1;           // multiplier
    while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1);
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;       // divider

    // timer for interrupt
    CpuTimer0Regs.TCR.bit.TSS = 1;
    CpuTimer0Regs.PRD.all = 1.25e6 - 1;  // T=0.125s from fclk
    CpuTimer0Regs.TCR.bit.TRB = 1;
    CpuTimer0Regs.TCR.bit.TIE = 1;

    // interrupt setup
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.TINT0 = &blinkISR;     // attach blink to timer0
    PieVectTable.XINT1 = &pressISR;     // attach button press to external
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // enable blink
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;  // enable press
    IER = 1;

    // external interrupt for gpio12
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 12;  // Interrupt 1 on gpio12
    XIntruptRegs.XINT1CR.bit.POLARITY = 0b01;   // rising edge
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;

    CpuTimer0Regs.TCR.bit.TSS = 0;
    SysCtrlRegs.WDCR = 0x28; EDIS; EINT;
    // ===============================End Setup================================

	while (1) {
	    SysCtrlRegs.WDKEY = 0x55;
	    SysCtrlRegs.WDKEY = 0xAA;
	    btstate = GpioDataRegs.GPADAT.bit.GPIO12;
	}
}


/** blinkISR
 *
 * Blinks LEDs according to the current state.
 *
 * Timer interrupt should have T=0.125s (a period of 0.125s).
 *
 * The shortest blink period is 0.125s, and the other state
 * periods are integer multipes of 0.125s, so it makes sense
 * to use a counter (loop_count) as a software clock divider
 * to enable different periods of action without constantly
 * re-setting-up the interrupt.
 *
 * s0: gpio0, T=1s
 * s1: gpio1, T=0.5s
 * s2: gpio2, T=0.25s
 * s3: gpio3, T=0.125s
 */
interrupt void blinkISR(void)
{
    switch (s) {
    // 1s
    case 0:
        if (loop_count % 4 == 0) GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
        break;
    // 0.5s
    case 1:
        if (loop_count % 3 == 0) GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;
        break;
    // 0.25s
    case 2:
        if (loop_count % 2 == 0) GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1;
        break;
    // 0.125s
    case 3:
        GpioDataRegs.GPATOGGLE.bit.GPIO3 = 1;
        break;
    default:
//        s %= 4;
        break;
    }

    // increment loop count every interrupt period
    // overflowing is acceptable, as the blink logic
    // already uses modulus arithmetic
    loop_count++;

    PieCtrlRegs.PIEACK.all = M_INT1;
}//*/


/** pressISR
 *
 * Transitions state when button (gpio12) is pressed
 *
 * Simple series-overflow transition:
 *
 *      s0 -> s1 -> s2 -> s3
 *      ^                 |
 *      |-----------------|
 *
 * Triggers on rising edge
 */
interrupt void pressISR(void)
{
    // state change
    s++;     // increment state
    s %= 4;  // s3->s0 overflow reset

    // reset loop count
    loop_count = 0;

    // turn off all LEDs going into next state
    GpioDataRegs.GPASET.bit.GPIO0 = 1;
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    GpioDataRegs.GPASET.bit.GPIO2 = 1;
    GpioDataRegs.GPASET.bit.GPIO3 = 1;

    PieCtrlRegs.PIEACK.all = M_INT1;
}
