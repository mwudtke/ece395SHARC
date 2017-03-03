/*****************************************************************************
 * dmaTest.c
 *****************************************************************************/

#include <Cdef21489.h>
#include <signal.h>
#include <stdio.h>
 
// Check SRU Routings for errors.
#define SRUDEBUG
#include <SRU.h>

#define SELECT_SPI_SLAVE(select) (*pSPIFLG &= ~(DS0EN<<8))
#define DESELECT_SPI_SLAVE(select) (*pSPIFLG |= (DS0EN<<8))

#define HIGHEST         0x7FFFFF00
#define LOWEST          0xFFFFFF00
#define SIGNAL_LENGTH   64

// Addresses
#define AK4396_ADDR    (0x00)

#define AK4396_CTRL1   (0x00)
#define AK4396_CTRL2   (0x01)
#define AK4396_CTRL3   (0x02)
#define AK4396_LCH_ATT (0x03)
#define AK4396_RCH_ATT (0x04)

// Reset CTRL1 setting
#define AK4396_CTRL1_RST   (0x86)

// Default settings
#define AK4396_CTRL1_DEF   (0x87)
#define AK4396_CTRL2_DEF   (0x02)
#define AK4396_CTRL3_DEF   (0x00)
#define AK4396_LCH_ATT_DEF (0xFF)
#define AK4396_RCH_ATT_DEF (0xFF)

// Configure the PLL for a core-clock of 266MHz and SDCLK of 133MHz
extern void initPLL_SDRAM(void);

// local functions
void initSRU(void);		
void initSPI(unsigned int SPI_Flag);
void configureAK4396Register(unsigned int address, unsigned int data);
void initDMA(void);
void testISR(int sig_int);
void delay(int times);

int rx0a_buf[2] = {0, 0};		// SPORT0 receive buffer a
int tx1a_buf[2] = {0, 0};		// SPORT1 transmit buffer a

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
int rx0a_tcb[8]  = {0, 0, 0, 0, 0, 2, 1, (int) rx0a_buf};				// SPORT0 receive a tcb from SPDIF
int tx1a_tcb[8]  = {0, 0, 0, 0, 0, 2, 1, (int) tx1a_buf};				// SPORT1 transmit a tcb to DAC


void main(void) {
	initPLL_SDRAM();

	/* schedule the interrupt service routine for when sport0 DMA is done */
	interrupts(SIG_SP0,testISR);

	initSPI(DS0EN);
	initSRU();
	
	//Set the reset so that the device is ready to initialize registers.
	configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_RST);
	
	delay(10);
        	
    configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_DEF);
	configureAK4396Register(AK4396_CTRL2, AK4396_CTRL2_DEF);
	configureAK4396Register(AK4396_CTRL3, AK4396_CTRL3_DEF);
	configureAK4396Register(AK4396_LCH_ATT, AK4396_LCH_ATT_DEF);
	configureAK4396Register(AK4396_RCH_ATT, AK4396_RCH_ATT_DEF);
	
	initDMA();

	//debug
	//printf("tx0a_tcb[4] = %x\n", tx0a_tcb[4]);
	//printf("CPSP0A = %x\n", *pCPSP0A);
	//printf("tx0a_buf address = %x\n", tx0a_buf);

	/* stream the signal to the DAC forever */
	while(1);
}

void initSRU() {
	
	// use pin 11 on the board for SPDIF in
	// this is the pin closest to the power,
	// in the not-ground row
	SRU(DAI_PB11_O, DIR_I);

	//Power off the DAC
	SRU(HIGH, DPI_PBEN04_I);
	SRU(LOW, DPI_PB04_I);
	
	delay(10);
	
	//Attach Main Clocks from SPDIF receiver
	
	//MCLK
	//SRU(PCG_CLKA_O, DAI_PB05_I);
	SRU(DIR_TDMCLK_O, DAI_PB05_I);
	SRU(HIGH,PBEN05_I);
	
	//BICK
	//SRU(PCG_CLKB_O, DAI_PB06_I);
	SRU(DIR_CLK_O, DAI_PB06_I);
	SRU(HIGH,PBEN06_I);
	
	//LRCK
	//SRU(PCG_CLKC_O, DAI_PB03_I);
	SRU(DIR_FS_O, DAI_PB03_I);
	SRU(HIGH,PBEN03_I);
	
	//CSN
	SRU(SPI_FLG0_PBEN_O, DPI_PBEN07_I);
	SRU(SPI_FLG0_O, DPI_PB07_I);
	
	//Set MOSI/CDT1 to output
	SRU(SPI_MOSI_O, DPI_PB01_I);
	SRU(HIGH, DPI_PBEN01_I);
	
	//Send SPI clock to DPI 3
	SRU(SPI_CLK_O, DPI_PB03_I);
	SRU(SPI_CLK_PBEN_O, DPI_PBEN03_I);
	
	//Power back on the DAC
	SRU(HIGH, DPI_PB04_I);
	
	delay(10);
	
	//SRU(PCG_CLKB_O, SPORT0_CLK_I);	// BICK is clock for SPORT0
	SRU(DAI_PB06_O, SPORT0_CLK_I);
	SRU(DAI_PB06_O, SPORT1_CLK_I);
	SRU(DAI_PB03_O, SPORT0_FS_I);
	SRU(DAI_PB03_O, SPORT1_FS_I);

	// SPORT0 receives from SPDIF
	SRU(DIR_DAT_O, SPORT0_DA_I);

	// SPORT1 outputs to the DAC
	SRU(SPORT1_DA_O, DAI_PB04_I);
	SRU(HIGH, PBEN04_I);
	
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
    unsigned int message = 0;

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

	rx0a_tcb[4] = *pCPSP0A = ((int) tx1a_tcb  + 7) & 0x7FFFF | (1<<19);
	tx1a_tcb[4] = *pCPSP1A = ((int) rx0a_tcb  + 7) & 0x7FFFF | (1<<19);

	//*pCPSP0A = ((int) tx0a_tcb + 7) & 0x7FFFF;
	
	// SPORT0 as receiver
	*pSPCTL0 = OPMODE | L_FIRST | SLEN32 | SPEN_A | SCHEN_A | SDEN_A;

	// SPORT1 as transmitter
	*pSPCTL1 = OPMODE | L_FIRST | SLEN32 | SPEN_A | SCHEN_A | SDEN_A | SPTRAN;			// Configure the SPORT control register
	//*pSPCTLN0 |= (1 << 1);
}

/* 
 * once we have received new SPDIF data into SPORT0,
 * transfer it to the SPORT1 transmit buffer to the DAC
 */
void testISR(int sig_int)
{
	//printf("processing time %d\n", processNum);

	tx1a_buf[0] = rx0a_buf[0];
	tx1a_buf[1] = rx0a_buf[1];
	
	printf("tx1a_buf is %x\n", tx1a_buf[0]);
}

void delay(int times)
{
	int i;

    for(i = times; i > 0; --i)
    	asm("nop;");
}
