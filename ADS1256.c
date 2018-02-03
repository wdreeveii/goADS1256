/*
			 define from bcm2835.h						 define from Board DVK511
				 3.3V | | 5V			   ->				  3.3V | | 5V
	RPI_V2_GPIO_P1_03 | | 5V			   ->				   SDA | | 5V 
	RPI_V2_GPIO_P1_05 | | GND			   ->				   SCL | | GND
	   RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->				   IO7 | | TX
				  GND | | RPI_GPIO_P1_10   ->				   GND | | RX
	   RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->				   IO0 | | IO1
	RPI_V2_GPIO_P1_13 | | GND			   ->				   IO2 | | GND
	   RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->				   IO3 | | IO4
				  VCC | | RPI_GPIO_P1_18   ->				   VCC | | IO5
	   RPI_GPIO_P1_19 | | GND			   ->				  MOSI | | GND
	   RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->				  MISO | | IO6
	   RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->				   SCK | | CE0
				  GND | | RPI_GPIO_P1_26   ->				   GND | | CE1

::if your raspberry Pi is version 1 or rev 1 or rev A
RPI_V2_GPIO_P1_03->RPI_GPIO_P1_03
RPI_V2_GPIO_P1_05->RPI_GPIO_P1_05
RPI_V2_GPIO_P1_13->RPI_GPIO_P1_13
::
*/

#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "ADS1256.h"

static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/*reset the default values	*/
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};

void  bsp_DelayUS(uint64_t micros);
uint8_t ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
static uint8_t ADS1256_ReadReg(uint8_t _RegID);
static void ADS1256_SetChannal(uint8_t _ch);
static void ADS1256_SetDiffChannal(uint8_t _ch);
uint8_t ADS1256_WaitDRDY(void);
static int32_t ADS1256_ReadData(void);

void  bsp_DelayUS(uint64_t micros)
{
		bcm2835_delayMicroseconds (micros);
}


/*
*********************************************************************************************************
*	name: ADS1256_CfgADC
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain:gain 1-64
*					   _drate:	data  rate
*	The return value: error
*********************************************************************************************************
*/
uint8_t ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{

	if ( ADS1256_WaitDRDY() > 0 ) return 1;

	{
		uint8_t buf[4];		/* Storage ads1256 register configuration parameters */

		/*Status register define
			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

			Bit 3 ORDER: Data Output Bit Order
				0 = Most Significant Bit First (default)
				1 = Least Significant Bit First
			Input data	is always shifted in most significant byte and bit first. Output data is always shifted out most significant
			byte first. The ORDER bit only controls the bit order of the output data within the byte.

			Bit 2 ACAL : Auto-Calibration
				0 = Auto-Calibration Disabled (default)
				1 = Auto-Calibration Enabled
			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
			values.

			Bit 1 BUFEN: Analog Input Buffer Enable
				0 = Buffer Disabled (default)
				1 = Buffer Enabled

			Bit 0 DRDY :  Data Ready (Read Only)
				This bit duplicates the state of the DRDY pin.

			ACAL=1	enable	calibration
		*/
		//buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer
		buf[0] = (0 << 3) | (1 << 2) | (0 << 1);  // The internal buffer is prohibited

		//ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;	

		/*	ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 ¦Ì A
				10 = Sensor Detect Current = 2 ¦Ì A
				11 = Sensor Detect Current = 10¦Ì A
				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
				000 = 1 (default)
				001 = 2
				010 = 4
				011 = 8
				100 = 16
				101 = 32
				110 = 64
				111 = 64
		*/
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	

		bcm2835_spi_transfer(CMD_WREG | 0); /* Write command register, send the register address */
		bcm2835_spi_transfer(0x03);         /* Register number 4,Initialize the number	-1*/

		bcm2835_spi_transfer(buf[0]);       /* Set the status register */
		bcm2835_spi_transfer(buf[1]);       /* Set the input channel parameters */
		bcm2835_spi_transfer(buf[2]);       /* Set the ADCON control register,gain */
		bcm2835_spi_transfer(buf[3]);       /* Set the output rate */

	}

	bsp_DelayUS(50);

	return 0;
}


/*
*********************************************************************************************************
*	name: ADS1256_WriteReg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	bcm2835_spi_transfer(CMD_WREG | _RegID);	/*Write command register */
	bcm2835_spi_transfer(0x00);		/*Write the register number */

	bcm2835_spi_transfer(_RegValue);	/*send register value */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadReg
*	function: Read	the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	bcm2835_spi_transfer(CMD_RREG | _RegID);	/* Write command register */
	bcm2835_spi_transfer(0x00);	/* Write the register number */
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(10);	/*delay time */

	read = bcm2835_spi_transfer(0xff);	/* Read the register values */

	return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_SetChannal
*	function: Configuration channel number
*	parameter:	_ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ¡°don¡¯t care¡±)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ¡°don¡¯t care¡±)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}

/*
*********************************************************************************************************
*	name: ADS1256_SetDiffChannal
*	function: The configuration difference channel
*	parameter:	_ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ¡°don¡¯t care¡±)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ¡°don¡¯t care¡±)
	*/
	if (_ch == 0) {
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* DiffChannal	AIN0£¬ AIN1 */
	}
	else if (_ch == 1) {
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/*DiffChannal	AIN2£¬ AIN3 */
	}
	else if (_ch == 2) {
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/*DiffChannal	 AIN4£¬ AIN5 */
	}
	else if (_ch == 3) {
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/*DiffChannal	AIN6£¬ AIN7 */
	}
}

/*
*********************************************************************************************************
*	name: ADS1256_WaitDRDY
*	function: delay time  wait for automatic calibration
*	parameter:	NULL
*	The return value:  NULL
*********************************************************************************************************
*/
uint8_t ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 400000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 400000)
	{
		// ADS1256_WaitDRDY() Time Out ...
		return 1;		
	}

	return 0;
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadData
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
	static uint8_t buf[3];


	bcm2835_spi_transfer(CMD_RDATA);	/* read ADC command  */
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(10);	/*delay time  */

	/*Read the sample results 24bit*/
	buf[0] = bcm2835_spi_transfer(0xff);
	buf[1] = bcm2835_spi_transfer(0xff);
	buf[2] = bcm2835_spi_transfer(0xff);

	read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong	read |= (buf[1] << 8) */
	read |= buf[2];


	/* Extend a signed number*/
	if (read & 0x800000)
	{
		read |= 0xFF000000;
	}

	return (int32_t)read;
}

/*
*********************************************************************************************************
*	name: ADS1256_Collect
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
int32_t ADS1256_Collect(uint8_t scanmode, uint8_t channel) {
	while(!DRDY_IS_LOW());
	CS_0();
	
	if (scanmode == 0) {	/*	0  Single-ended input  8 channel£¬ 1 Differential input  4 channe */

		ADS1256_SetChannal(channel);	/*Switch channel mode */
		bsp_DelayUS(5);

		bcm2835_spi_transfer(CMD_SYNC);
		bsp_DelayUS(5);

		bcm2835_spi_transfer(CMD_WAKEUP);
		bsp_DelayUS(25);

		return ADS1256_ReadData();

	} else {	/*DiffChannal*/
		
		ADS1256_SetDiffChannal(channel);	/* change DiffChannal */
		bsp_DelayUS(5);

		bcm2835_spi_transfer(CMD_SYNC);
		bsp_DelayUS(5);

		bcm2835_spi_transfer(CMD_WAKEUP);
		bsp_DelayUS(25);

		return ADS1256_ReadData();
	}
}

int ADS1256_init(void) {
	uint8_t id;

	if (!bcm2835_init()) return 1;

	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );	   // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);					  // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024); // The default
	bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
	bcm2835_gpio_write(SPICS, HIGH);
	bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);		
	CS_0();		
	//ADS1256_WriteReg(REG_MUX,0x01);
	//ADS1256_WriteReg(REG_ADCON,0x20);
	// ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_15SPS);
	
	if ( ADS1256_WaitDRDY() > 0) return 2;

	id = ADS1256_ReadReg(REG_STATUS);
	id = id >> 4;

	if (id != 3) {
		printf("Error, ASD1256 Chip ID = 0x%d\r\n", (unsigned int) id);
		return 3;
	}
	
	if ( ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_15SPS) > 0 ) return 4;

	return 0;
}

void ADS1256_dest(void) {

	bcm2835_spi_end();
	bcm2835_close();

}

/*
int main()
{
	int32_t adc[8];
	int32_t volt[8];
	uint8_t i;
	uint8_t ch_num;
	int32_t iTemp;
	uint8_t buf[3];
	
	int init_status = ADS1256_init();
	if ( init_status > 0 ) {
		printf("ADS1256_init() returned: %d\n", init_status);
		return init_status;
	}

	ch_num = 8;	
	while(1) {
		
		for (i = 0; i < ch_num; i++) {
			adc[i] = ADS1256_Collect(0, i);
			volt[i] = (adc[i] * 100) / 167;	
		}
		
		for (i = 0; i < ch_num; i++) {
			buf[0] = ((uint32_t)adc[i] >> 16) & 0xFF;
			buf[1] = ((uint32_t)adc[i] >> 8) & 0xFF;
			buf[2] = ((uint32_t)adc[i] >> 0) & 0xFF;
			printf("%d=%02X%02X%02X, %8ld", (int)i, (int)buf[0], (int)buf[1], (int)buf[2], (long)adc[i]);				   

			iTemp = volt[i];	// uV
			if (iTemp < 0) {
				iTemp = -iTemp;
				printf(" (-%ld.%03ld %03ld V) \r\n", iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);
			}
			else {
				printf(" ( %ld.%03ld %03ld V) \r\n", iTemp /1000000, (iTemp%1000000)/1000, iTemp%1000);					   
			}
					
		}
		printf("\33[%dA", (int)ch_num);  
		bsp_DelayUS(1000);	
	}
	
	ADS1256_dest();
	
	return 0;
}

*/

