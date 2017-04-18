 /*****************************************************************************
 * dmaTest.c
 *****************************************************************************/

#include <Cdef21489.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
 
// Check SRU Routings for errors.
#define SRUDEBUG
#include <SRU.h>

#define SELECT_SPI_SLAVE(select) (*pSPIFLG &= ~(DS0EN<<8))
#define DESELECT_SPI_SLAVE(select) (*pSPIFLG |= (DS0EN<<8))

// Addresses
#define AK4396_ADDR    (0x00)

#define AK4396_CTRL1   (0x00)
#define AK4396_CTRL2   (0x01)
#define AK4396_CTRL3   (0x02)
#define AK4396_LCH_ATT (0x03)
#define AK4396_RCH_ATT (0x04)

// Reset CTRL1 setting
#define AK4396_CTRL1_RST   (0x06)

// Default settings
#define AK4396_CTRL1_DEF   (0x87)
#define AK4396_CTRL2_DEF   (0x02)
#define AK4396_CTRL3_DEF   (0x00)
#define AK4396_LCH_ATT_DEF (0xFF)
#define AK4396_RCH_ATT_DEF (0xFF)

#define BUFFER_LENGTH 1024
#define BUFFER_MASK 0x000000FF

#define PI 3.141592653589793238462643

// Configure the PLL for a core-clock of 266MHz and SDCLK of 133MHz
extern void initPLL_SDRAM(void);

// local functions
void initSRU(void);
void initSPI(unsigned int SPI_Flag);
void configureAK4396Register(unsigned int address, unsigned int data);
void initDMA(void);
void initSPDIF(void);
void clearDAIpins(void);
void initPCG(void);
void processSamples(void);
void initWindow(void);

void delay(int times);

int rx0a_buf[BUFFER_LENGTH] = {0};		// SPORT0 receive buffer a - also used for transmission
int process_buf1[BUFFER_LENGTH] = {0};
int process_buf2[BUFFER_LENGTH] = {0};
int output_buf[BUFFER_LENGTH] = {0};
int tx1a_buf_dummy[BUFFER_LENGTH/2] = {0};
/* TCB format:    ECx (length of destination buffer),
				  EMx (destination buffer step size),
				  EIx (destination buffer index (initialized to start address)),
				  GPx ("general purpose"),
				  CPx ("Chain Point register"; points to last address (IIx) of
			   								   next TCB to jump to
				                               upon completion of this TCB.),
				  Cx  (length of source buffer),
				  IMx (source buffer step size),
				  IIx (source buffer index (initialized to start address))       */
int rx0a_tcb[8]  = {0, 0, 0, 0, 0, BUFFER_LENGTH, 1, (int) rx0a_buf};				// SPORT0 receive a tcb from SPDIF
int tx1a_tcb[8]  = {0, 0, 0, 0, 0, BUFFER_LENGTH, 1, (int) rx0a_buf};				// SPORT1 transmit a tcb to DAC

int tx1a_delay_tcb[8]  = {0, 0, 0, 0, 0, BUFFER_LENGTH/2, 1, (int) tx1a_buf_dummy};				// SPORT1 transmit a tcb to DAC

int dsp = 0;
double hann_window[BUFFER_LENGTH];

unsigned int mclk_divider = 2;
unsigned int clkdiv = 17;  // higher number == lower sample rate; vice versa. can get nicely arbitrary sample rates. GOT MATH?
unsigned int fsdiv = 63;  // always?


void main(void) {

	unsigned int max = clkdiv + 10;
	unsigned int min = clkdiv - 10;
	int adj = 1;
	unsigned int count = 0;

	initWindow();

	initPLL_SDRAM();

	initSPI(DS0EN);

	initPCG();

	initSRU();
	

	//Set the reset so that the device is ready to initialize registers.
	configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_RST);
	delay(10);
        	
    configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_DEF);
	delay(10);

	configureAK4396Register(AK4396_CTRL2, AK4396_CTRL2_DEF);
	delay(10);

	configureAK4396Register(AK4396_CTRL3, AK4396_CTRL3_DEF);
	delay(10);
	
	configureAK4396Register(AK4396_LCH_ATT, AK4396_LCH_ATT_DEF);
	delay(10);
	
	configureAK4396Register(AK4396_RCH_ATT, AK4396_RCH_ATT_DEF);
	delay(10);

	initDMA();

	initSPDIF();

	/* stream the signal to the DAC forever */
	while(1){
		//processSamples();

		/*
		if (count == 150000) {

			//printf("cldiv == %d\n",clkdiv);
			if (clkdiv > max) 
				adj = -1;
			else if (clkdiv < min)
				adj = 1; 

			clkdiv = clkdiv + adj;
		
			*pDIV1 = (clkdiv << 1) | (fsdiv << 16);

			count = 0;
		}
		
		count++;
		*/
	}  
}

void initSPDIF()
{
    // SPDIF Setup code goes here
    // Use default setting of SPDIF
    *pDIRCTL=0x0;
    
}


void initSRU() {

	clearDAIpins();
	
	// use pin 12 on the board for SPDIF in
	// this is the pin second from the power,
	// in the not-ground row
	SRU(LOW, DAI_PB12_I);
	SRU(LOW, PBEN12_I);
	SRU(DAI_PB12_O, DIR_I);

	//trigger the pcg??
	SRU(SPORT1_FS_O, DAI_PB02_I);
	SRU(HIGH, PBEN02_I);
	SRU(DAI_PB02_O, PCG_SYNC_CLKA_I);

	//Power off the DAC
	SRU2(HIGH, DPI_PBEN04_I);
	SRU2(LOW, DPI_PB04_I);
	
	delay(10);
	
	//Attach Main Clocks from SPDIF receiver
	
	//MCLK
	// old way with spdif mclock SRU(DIR_TDMCLK_O, DAI_PB05_I);
	SRU(PCG_FSA_O, DAI_PB05_I);
	SRU(HIGH,PBEN05_I);
	
	//BICK
	// old way with spdif SRU(DIR_CLK_O, DAI_PB06_I);
	SRU(SPORT1_CLK_O, DAI_PB06_I);
	SRU(HIGH,PBEN06_I);
	
	//LRCK
	// old way with spdif SRU(DIR_FS_O, DAI_PB03_I);
	SRU(SPORT1_FS_O, DAI_PB03_I);
	SRU(HIGH,PBEN03_I);
	
	//CSN
	SRU2(SPI_FLG0_O, DPI_PB07_I);
    SRU2(SPI_FLG0_PBEN_O, DPI_PBEN07_I);
    //SRU2(HIGH, DPI_PBEN07_I);
	
	//Set MOSI/CDT1 to output
	SRU2(SPI_MOSI_O, DPI_PB01_I);
	SRU2(HIGH, DPI_PBEN01_I);
	
	//Send SPI clock to DPI 3
	SRU2(SPI_CLK_O, DPI_PB03_I);
	SRU2(SPI_CLK_PBEN_O, DPI_PBEN03_I);
	
	//Power back on the DAC
	SRU2(HIGH, DPI_PB04_I);

	delay(10);

	SRU(DIR_CLK_O, SPORT0_CLK_I);
	//SRU(DAI_PB06_O, SPORT1_CLK_I);
	SRU(DIR_FS_O, SPORT0_FS_I);
	//SRU(DAI_PB03_O, SPORT1_FS_I);

	// SPORT0 receives from SPDIF (comment back in to test talkthrough)
	SRU(DIR_DAT_O, SPORT0_DA_I);

	// SPORT1 outputs to the DAC (comment back in to test talkthrough)
	SRU(SPORT1_DA_O, DAI_PB04_I);
	SRU(HIGH, PBEN04_I);

	
	//DEBUG SIGNALS//
	
	// LRCLK to debug, pin 11
	SRU(PCG_FSA_O, DAI_PB11_I);
	SRU(HIGH, PBEN11_I);
	// MOSI to debug
    SRU2(SPORT1_FS_O, DAI_PB15_I);
    SRU2(HIGH, PBEN15_I);

}

// Serial Peripheral Interface Initialization
void initSPI(unsigned int SPI_Flag)
{
	// Configure the SPI Control registers
    // First clear a few registers
    *pSPICTL = (TXFLSH | RXFLSH) ; //Clear TXSPI and RXSPI
    *pSPIFLG = 0; //Clear the slave select

    // Setup the baud rate to 1MHz
    //*pSPIBAUD = 100;
    //BAUDR is bits 15-1 and 0 is reserved
    *pSPIBAUD = 25;   //SPICLK baud rate = PCLK / (4*BAUDR)
    //PCLK = 200MHz and 200MHz/100 = 2 MHz? - double check this...

    // Setup the SPI Flag register using the Flag specified in the call
    *pSPIFLG = (0xF00|SPI_Flag);

    // Now setup the SPI Control register
    *pSPICTL = (SPIEN | SPIMS | WL16 | MSBF | TIMOD1 | CLKPL|GM); //| SMLS | CPHASE
    
    //SPIEN - SPI System Enable
    //SPIMS - Master Slave Mode Bit - 1 indicates we are a master device
    //WL16 - Word length is 16 bits
    //MSBF - MSB sent/received first
    //(NOT) CPHASE - SPICLK starts toggling at the middle of 1st data bit
    //CLKPL - Clock Polarity - Active low SPICLK (SPICLK high is the idle state)
    //(NOT) SMLS - Seamless Transfer disabled - delay before the next word starts, done to ensure frame on osciliscope, but check if you can change this.
    //GM - Get Data. When RXSPI is full, get more data which overwrites previous data.
}

void configureAK4396Register(unsigned int address, unsigned int data)
{
    unsigned short message = 0;

    SELECT_SPI_SLAVE(DS0EN);

    ///MSB                                LSB
    //[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]

    //Read/Write (stay at 1, write only, as per diagram).
    message |= 0x2000; //message[3]
    
    //Register Address
    message |= (address << 8);
        
    message |= data;
    
    *pTXSPI = message;

    //Wait for the SPI to indicate that it has finished.
    while ((*pSPISTAT & TXS))

    //Wait for the SPI to indicate that it has finished.
    while (!(*pSPISTAT & SPIFE))
    
	delay(10);
    
    DESELECT_SPI_SLAVE(DS0EN);

}

void initDMA() {

	*pSPMCTL0 = 0; // ******* ONLY SET ONCE 
	*pSPMCTL1 = 0;
	
	*pSPCTL0 = 0;
	*pSPCTL1 = 0;

	//comment back in to test sport talkthrough
	rx0a_tcb[4] = *pCPSP0A = ((int) rx0a_tcb  + 7) & 0x7FFFF | (1<<19);
	tx1a_tcb[4] = ((int) tx1a_delay_tcb  + 7) & 0x7FFFF | (1<<19);
	tx1a_delay_tcb[4] = *pCPSP1A = ((int) tx1a_tcb  + 7) & 0x7FFFF | (1<<19);


	// SPORT0 as receiver (SPTRAN for testing square wave)
	// OPMODE = I2S mode!
	*pSPCTL0 = OPMODE | L_FIRST | SLEN24 | SPEN_A | SCHEN_A | SDEN_A;

	// SPORT1 as transmitter
	*pSPCTL1 = OPMODE | L_FIRST | SLEN24 | SPEN_A | SCHEN_A | SDEN_A | SPTRAN | MSTR;			// Configure the SPORT control register

	*pDIV1 = (clkdiv << 1) | (fsdiv << 16);
}

void delay(int times)
{
	int i;

    for(i = times; i > 0; --i)
    	asm("nop;");
}

void clearDAIpins(void)
{
//------------------------------------------------------------------------
//  Tie the pin buffer inputs LOW for all DAI pins.  Even though
//    these pins are inputs to the SHARC, tying unused pin buffer inputs
//    LOW is "good coding style" to eliminate the possibility of
//    termination artifacts internal to the IC.  Note that signal
//    integrity is degraded only with a few specific SRU combinations.
//    In practice, this occurs VERY rarely, and these connections are
//    typically unnecessary.  This is GROUP D
    SRU(LOW, DAI_PB01_I);
    SRU(LOW, DAI_PB02_I);
    SRU(LOW, DAI_PB03_I);
    SRU(LOW, DAI_PB04_I);
    SRU(LOW, DAI_PB05_I);
    SRU(LOW, DAI_PB06_I);
    SRU(LOW, DAI_PB07_I);
    SRU(LOW, DAI_PB08_I);
    SRU(LOW, DAI_PB09_I);
    SRU(LOW, DAI_PB10_I);
    SRU(LOW, DAI_PB11_I);
    SRU(LOW, DAI_PB12_I);
    SRU(LOW, DAI_PB13_I);
    SRU(LOW, DAI_PB14_I);
    SRU(LOW, DAI_PB15_I);
    SRU(LOW, DAI_PB16_I);
    SRU(LOW, DAI_PB17_I);
    SRU(LOW, DAI_PB18_I);
    SRU(LOW, DAI_PB19_I);
    SRU(LOW, DAI_PB20_I);

//------------------------------------------------------------------------
//  Tie the pin buffer enable inputs LOW for all DAI pins so
//  that they are always input pins.  This is GROUP F.
    SRU(LOW, PBEN01_I);
    SRU(LOW, PBEN02_I);
    SRU(LOW, PBEN03_I);
    SRU(LOW, PBEN04_I);
    SRU(LOW, PBEN05_I);
    SRU(LOW, PBEN06_I);
    SRU(LOW, PBEN07_I);
    SRU(LOW, PBEN08_I);
    SRU(LOW, PBEN09_I);
    SRU(LOW, PBEN10_I);
    SRU(LOW, PBEN11_I);
    SRU(LOW, PBEN12_I);
    SRU(LOW, PBEN13_I);
    SRU(LOW, PBEN14_I);
    SRU(LOW, PBEN15_I);
    SRU(LOW, PBEN16_I);
    SRU(LOW, PBEN17_I);
    SRU(LOW, PBEN18_I);
    SRU(LOW, PBEN19_I);
    SRU(LOW, PBEN20_I);
}

// Precision Clock Generator Initialization
void initPCG(void)
{
	//CLKIN = 24.576 MHz (25MHz)
		
	// turn off the bit it said to turn off to enable external trigger thing
	*pPCG_SYNC1 &= ~(0x01);

	//MCLK
	*pPCG_CTLA0 = mclk_divider | ENFSA; // CLKADIV = 1, full 25 MHz
	//*pPCG_CTLA1 = mclk_divider;
	//*pPCG_CTLA0 = ENCLKA;

}

void processSamples() {


	while( ( ((int)rx0a_buf + dsp) & BUFFER_MASK ) != ( *pIISP0A & BUFFER_MASK ) ) {

		process_buf1[dsp] = rx0a_buf[dsp] * hann_window[dsp];

		int dsp2 = (dsp + BUFFER_LENGTH / 2) % BUFFER_LENGTH;

		process_buf2[dsp] = rx0a_buf[dsp2] * hann_window[dsp2];

		output_buf[dsp] = process_buf1[dsp] + process_buf2[dsp];

		//maybe?
		output_buf[dsp] ^= 0x80000000;

    	dsp = (dsp + 1) % BUFFER_LENGTH;
	}

    return;
}

/* initialize the hanning window */
void initWindow ()
{
	int i;

	for (i = 0; i < BUFFER_LENGTH; i++)
	    hann_window[i] = 0.5 * (1 - cos( (2 * PI * i) / (BUFFER_LENGTH - 1) ));
}
