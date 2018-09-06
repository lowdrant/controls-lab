/**
 * Task 1 for ECE4550 Fall 2018
 *
 * Author: Marion Anderson, Aditya Retnanto
 * Date: 2018-08-27
 * File: main.c
 */
#include "F2837xD_device.h"
unsigned long slow = 0, fast = 0;  // counters to view program execution


int main(void)
{
    // Turn Watchdog off & back on, per lab instructions
    // 'off' bits: 00000000 | 0 | 1 | 101 | 000
    // 'on' bits:  00000000 | 0 | 0 | 101 | 000
    //
    // Bits 6 thru 3 are the important ones.
    //     bit 6 turns watchdog on (0) and off (1)
    //     bits 5-3 are an error check. Must be '101' every time WDCR is written. Else reset
    EALLOW;
    WdRegs.WDCR.all = 0x68;
    WdRegs.WDCR.all = 0x28;
    EDIS;

	while (1) {
	    fast++;  // increment fast every loop
	    if (fast % 100000 == 0) {  // increment slow once every 10^5 loops
	        slow++;
	    }

	    // Reset watchdog constantly
        EALLOW;
	    WdRegs.WDKEY.all = 0x55;
	    WdRegs.WDKEY.all = 0xAA;
        EDIS;
	}
}
