/*****************************************************************************
 * dmaTest.c
 *****************************************************************************/

#include <Cdef21489.h>
#include <signal.h>
#include <stdio.h>
 
#define SRUDEBUG  				  							// Check SRU Routings for errors.
#include <SRU.h>

extern void initPLL_SDRAM(void); 							// Configure the PLL for a core-clock of 266MHz and SDCLK of 133MHz

void initSRU(void);		
void initDMA(void);
void initBuffer(void);
void initPCG(void);

int tx0a_buf0[10] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1};											// SPORT0 transmit buffer A
int tx0a_buf1[10];
int rx1a_buf[10];		   									// SPORT1 receive buffer A

int tx0a_tcb0[4]  = {0, 10, 1, (int) tx0a_buf0};	// SPORT0 transmit a tcb for zeros
int tx0a_tcb1[4]  = {0, 10, 1, (int) tx0a_buf1};	// SPORT0 transmit a tcb for ones
int rx1a_tcb[4]   = {0, 10, 1, (int) rx1a_buf};		// SPORT1 receive a tcb


void main(void) {
	
	initPLL_SDRAM();

	initBuffer();
	initPCG();
	initSRU();
	initDMA();
	
	while(1) {
		
		printf("rx1a_buf[3]==%x\n",rx1a_buf[3]);	
	}
	//return 0;
}

void initBuffer() {

	int i = 0;
	
	for(i=0; i<10; i++) {

		//tx0a_buf0[i] = 0x0;
		//tx0a_buf1[i] = 0x1;
	}	
}

void initSRU() {
		
 	SRU(SPORT0_DA_O, SPORT1_DA_I);
 	SRU(SPORT0_DA_O, DAI_PB11_I);
 	SRU(HIGH, PBEN11_I);
 	
	SRU(PCG_CLKA_O, SPORT0_CLK_I);			// Attach BICK as serial clock for SPORT0 
	SRU(PCG_CLKA_O, SPORT1_CLK_I);			
	
	SRU(PCG_CLKB_O, DAI_PB10_I);            // Frame-sync - for some reason, need to go through DAI with this...
	SRU(HIGH,PBEN10_I); 
	SRU(DAI_PB10_O, SPORT0_FS_I);
	SRU(DAI_PB10_O, SPORT1_FS_I);
}

void initDMA() {

	*pSPMCTL0 = 0;
	*pSPMCTL1 = 0;
		
	*pSPCTL0 = 0;
	*pSPCTL1 = 0;
	
	tx0a_tcb0[0]  = *pCPSP0A = ((int) tx0a_tcb0  + 3) & 0x7FFFF | (1<<19);
	//tx0a_tcb0[0]  = *pCPSP0A = ((int) tx0a_tcb1  + 3) & 0x7FFFF | (1<<19);				// Set up the chain pointer on all the sports
	//tx0a_tcb1[0]  = ((int) tx0a_tcb0  + 3) & 0x7FFFF | (1<<19);
	rx1a_tcb[0]  = *pCPSP1A = ((int) rx1a_tcb  + 3) & 0x7FFFF | (1<<19);
	
	
	*pSPCTL0 = OPMODE | L_FIRST | SLEN24 | SPEN_A | SCHEN_A | SDEN_A | SPTRAN;			// Configure the SPORT control register
	*pSPCTL1 = OPMODE | L_FIRST | SLEN24 | SPEN_A | SCHEN_A | SDEN_A;
	
}

void initPCG(void)							// Precision Clock Generator Initialization
{
	//CLKIN = 24.576 MHz (25MHz)
	
	//BICK
	*pPCG_CTLA1 = 8; 						// CLKBDIV = 8
	*pPCG_CTLA0 = ENCLKA;
	
	//LRCK
	*pPCG_CTLB1 = 512; 						// CLKCDIV = 512, for 48 kHz
	*pPCG_CTLB0 = ENCLKB;

	
}
