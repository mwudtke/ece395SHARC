/*****************************************************************************
 * dmaTest.c
 *****************************************************************************/

#include <Cdef21489.h>
#include <signal.h>
#include <stdio.h>
 
#define SRUDEBUG  				  							// Check SRU Routings for errors.
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

extern void initPLL_SDRAM(void); 							// Configure the PLL for a core-clock of 266MHz and SDCLK of 133MHz

void initSRU(void);		
void initPCG(void);
void initSPI(unsigned int SPI_Flag);
void configureAK4396Register(unsigned int address, unsigned int data);
void initDMA(void);
void testISR(int sig_int);
void initSignal(void);

static int i = 0, sampleNum = 0;

static int count;

/* variables for interrupt handling */
static volatile int blockReady = 0, isProcessing = 0;

static int processNum = 0;



int source[SIGNAL_LENGTH];	// square wave signal


int tx0a_buf[2] = {0, 0};									// SPORT0 transmit buffer A

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
int tx0a_tcb[8]  = {0, 0, 0, 0, 0, 2, 1, (int) tx0a_buf};				// SPORT0 transmit a tcb for zeros

void main(void) {	
	initPLL_SDRAM();
	
	initSignal();
	
	/* schedule the interrupt service routine for when sport0 DMA is done */
	interrupts(SIG_SP0,testISR);

	initPCG();
	initSPI(DS0EN);
	initSRU();
	
	//Set the reset so that the device is ready to initialize registers.
	configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_RST);
	
	for(i=10; i > 0; --i)
    	asm("nop;");
        	
    configureAK4396Register(AK4396_CTRL1, AK4396_CTRL1_DEF);
	configureAK4396Register(AK4396_CTRL2, AK4396_CTRL2_DEF);
	configureAK4396Register(AK4396_CTRL3, AK4396_CTRL3_DEF);
	configureAK4396Register(AK4396_LCH_ATT, AK4396_LCH_ATT_DEF);
	configureAK4396Register(AK4396_RCH_ATT, AK4396_RCH_ATT_DEF);
	
	initDMA(); /** look **/


	//printf("tx0a_tcb[4] = %x\n", tx0a_tcb[4]);
	//printf("CPSP0A = %x\n", *pCPSP0A);
	//printf("tx0a_buf address = %x\n", tx0a_buf);


	/* stream the signal to the DAC forever */
	while(1) {

    }
}

void initSignal()
{
	int i;
	
	for (i = 0; i < SIGNAL_LENGTH / 2; i++)
	{
		source[i] = HIGHEST;	
	}
	
	for (i = SIGNAL_LENGTH / 2; i < SIGNAL_LENGTH; i++)
	{
		source[i] = LOWEST;	
	}
	
}

void initSRU() {
		
	//Power off the DAC
	
	SRU(HIGH, DPI_PBEN04_I);
	SRU(LOW, DPI_PB04_I);
	
	for(i=10; i > 0; --i)
        	asm("nop;");
	
	//Attach Main Clocks
	
	//MCLK
	SRU(PCG_CLKA_O, DAI_PB05_I);
	SRU(HIGH,PBEN05_I);
	
	//BICK
	SRU(PCG_CLKB_O, DAI_PB06_I);
	SRU(HIGH,PBEN06_I);
	
	//LRCK
	SRU(PCG_CLKC_O, DAI_PB03_I);
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

	// send spi clock out to DPI 11, which is an available pin on the dev board
	// connect DPI 11 to DAI 09, which is another available pin on the dev board
	// internally connect DAI 09 to DAI 07, which is hardwired to the CCLK pin
	// on the dac
	//SRU(SPI_CLK_O, DPI_PB11_I);
	//SRU(SPI_CLK_PBEN_O, DPI_PBEN11_I);
	//SRU(DAI_PB09_O, DAI_PB07_I);
	//SRU(LOW, PBEN09_I);
	//SRU(LOW, DAI_PB09_I);
	//SRU(HIGH, PBEN07_I);
	
	SRU(SPI_MOSI_PBEN_O, DPI_PBEN10_I);
	SRU(SPI_MOSI_O, DPI_PB10_I);
	
	//SRU(PCG_CLKC_O, DAI_PB12_I);
	//SRU(HIGH,PBEN12_I);
	
	//SRU(SPI_FLG0_PBEN_O, DPI_PBEN10_I);
	//SRU(SPI_FLG0_O, DPI_PB10_I);
	
	
	//SRU(SPI_CLK_O, DAI_PB07_I);
	//SRU(HIGH, PBEN07_I);            //Set CCLK clock to output
	
	//Power back on the DAC
	
	SRU(HIGH, DPI_PB04_I);
	
	for(i=10; i > 0; --i)
        	asm("nop;");
	
	SRU(PCG_CLKB_O, SPORT0_CLK_I);	// BICK is clock for SPORT0
	SRU(DAI_PB03_O, SPORT0_FS_I);

	SRU(SPORT0_DA_O, DAI_PB04_I);
	SRU(HIGH, PBEN04_I);
	
}

void initPCG(void)							// Precision Clock Generator Initialization
{
	//CLKIN = 24.576 MHz (25MHz)
	
	//MCLK
	*pPCG_CTLA1 = 0; // CLKADIV = 1, full 25 MHz
	*pPCG_CTLA0 = ENCLKA;
	
	//BICK
	*pPCG_CTLB1 = 8; // CLKBDIV = 8
	*pPCG_CTLB0 = ENCLKB;
	
	//LRCK
	*pPCG_CTLC1 = 512; // CLKCDIV = 512, for 48 kHz
	*pPCG_CTLC0 = ENCLKC;
	
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
    int i,j;
    unsigned int message = 0;

    SELECT_SPI_SLAVE(DS0EN);

    ///MSB                                LSB
    //[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
    
    //Chip Address (stay at 0 as per diagram).
    //message[0]= 0;
    //message[1]= 0;
    
    //Read/Write (stay at 1, write only, as per diagram).
    message |= 0x2000; //message[3]
    
    //Register Address
    message |= (address << 8);
        
    message |= data;
    
    j=0;
    *pTXSPI = message;
   // Delay(136);
    //Wait for the SPI to indicate that it has finished.
    while ((*pSPISTAT & TXS))
    {j++;} 

    j=0;
    //Wait for the SPI to indicate that it has finished.
    while (!(*pSPISTAT & SPIFE))
    {j++;}
    
    for(i=10; i > 0; --i)
        	asm("nop;");

    DESELECT_SPI_SLAVE(DS0EN);

}

void initDMA() {

	*pSPMCTL0 = 0; // ******* ONLY SET ONCE 
		
	*pSPCTL0 = 0;
	
	tx0a_tcb[4] = *pCPSP0A = ((int) tx0a_tcb  + 7) & 0x7FFFF | (1<<19);

	//*pCPSP0A = ((int) tx0a_tcb + 7) & 0x7FFFF;
	
	*pSPCTL0 = OPMODE | L_FIRST | SLEN32 | SPEN_A | SCHEN_A | SDEN_A | SPTRAN;			// Configure the SPORT control register
	
	*pSPCTLN0 |= (1 << 1);
}

void testISR(int sig_int)
{
	//printf("processing time %d\n", processNum);
	processNum++;

	tx0a_buf[0] = source[sampleNum];
	tx0a_buf[1] = source[sampleNum];
	
	sampleNum++;
	sampleNum %= SIGNAL_LENGTH;
}
