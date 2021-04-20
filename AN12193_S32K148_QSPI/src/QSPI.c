/*
 * QSPI.c
 *
 *  Created on: Mar 11, 2017
 *      Author: B55840
 */

#include "QSPI.h"
//#include "S32K148.h"

//This function Initializes the QSPI0

__attribute__((aligned(32)))  uint32_t  qspi_dest_cpu[256] ;


/* private functions */
static void quadspi_write_enable(void);
static void quadspi_set_lut(uint32_t index, uint32_t value);
static void launch_loot(uint8_t loot_index);
static void quadspi_wait_while_flash_busy();
static void setup_LUT_Q0(void);

//using internal DQS mode
void QSPI_setup()
{
	uint32_t i;

	/* Enable QuadSPI Clocks using PLL1 */
	/* Operate at a safe frequency initially */

	/* clock configuration */
	PCC-> PCCn[PCC_QSPI_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for QSPI */
	/*	pin configuration */

	/*************Basic config for Internal DQS Pad loopback***********/

	QuadSPI -> MCR |= QuadSPI_MCR_MDIS_MASK; /*Module disable*/
	QuadSPI -> SOCCR |= 0x10070000;/*Divider Disable*/
	QuadSPI -> MCR &= ~(QuadSPI_MCR_SCLKCFG_MASK);
	QuadSPI -> MCR |= 0x83000000; /*SCLKCFG7,1,0=1*/
	QuadSPI -> MCR &= ~(QuadSPI_MCR_DDR_EN_MASK);
	QuadSPI -> MCR |= QuadSPI_MCR_DQS_EN_MASK;
	QuadSPI -> SMPR  |= QuadSPI_SMPR_FSPHS_MASK ; /*Select sampling at inverted clock*/
	QuadSPI -> FLSHCR &= ~(QuadSPI_FLSHCR_TDH_MASK) ; /*Data aligned with the posedge of Internal reference clock of QuadSPI*/
	QuadSPI -> FLSHCR &= ~(QuadSPI_FLSHCR_TCSS_MASK) ;
	QuadSPI -> FLSHCR &= ~(QuadSPI_FLSHCR_TCSH_MASK) ;
	QuadSPI -> FLSHCR |= 0x102;
	QuadSPI -> SOCCR = 0x00070000;/*Divider Enable*/

	QuadSPI -> MCR &= ~(QuadSPI_MCR_END_CFG_MASK);
	QuadSPI -> MCR |= 0x08;

	QuadSPI -> MCR &= ~(QuadSPI_MCR_MDIS_MASK); /*Module Enable*/

	/**************Ungate Clock *********************************/
	SIM->MISCTRL0 |= 0x4000000;


	/*****************ADDRESS REGISTER SETTING*************************/
	QuadSPI-> SFA1AD = FLASH_A_BASE_ADDR + 0x4000000; 	/* Set top address of FA1 (size 512Mbit) */
	QuadSPI-> SFA2AD = FLASH_A_BASE_ADDR + 0x4000000;  	/* Set top address of FA2 (size 0Mbit) */
	QuadSPI-> SFB1AD = FLASH_B_BASE_ADDR + 0x4000000;  	/* Set top address of FB1 (size 512Mbit) */
	QuadSPI-> SFB2AD = FLASH_B_BASE_ADDR + 0x4000000;  	/* Set top address of FB2 (size 0Mbit) 0x203FFFFF */


	for(i=0;i<100;i++);

	/* Configure the Look Up Table*/
	setup_LUT_Q0();



	/**********Configuring AHB Buffer Size anf Master ID's*************************/
	QuadSPI-> BUF0IND = 0x00; /* buffer0 size 0 bytes */
	QuadSPI-> BUF1IND = 0x00; /* buffer1 size 0 bytes */
	QuadSPI-> BUF2IND = 0x00; /* buffer2 size 0 bytes */



	QuadSPI-> BUF0CR = 0xFF; /* buffer0  */
	QuadSPI-> BUF1CR = 0xFF; /* buffer1  */
	QuadSPI-> BUF2CR = 0xFF; /* buffer2  */
	QuadSPI-> BUF3CR = 0x80008000; /* buffer3  */



	for(i=0;i<100;i++);

	QuadSPI -> MCR |= QuadSPI_MCR_CLR_TXF_MASK;


	QuadSPI -> SFAR = 0x68000000;

	QuadSPI -> BFGENCR = 0x00000;
	for(i=0;i<100;i++);
}


void setup_LUT_Q0()
{
	/* Setup LUT */
	//Unlock the LUT
	QuadSPI -> LUTKEY = 0x5AF05AF0;
	QuadSPI -> LCKCR = 0x2;	//UNLOCK the LUT
	while(((QuadSPI -> LCKCR)&QuadSPI_LCKCR_UNLOCK_MASK)>>QuadSPI_LCKCR_UNLOCK_SHIFT == 0);

	/* SEQID 0 - Used for AHB reading */
	QuadSPI-> LUT[0] = 0x0A1804EB;
	QuadSPI-> LUT[1] = 0x1E800E06;
	QuadSPI-> LUT[2] = 0x00002400;
	QuadSPI-> LUT[3] = 0x00000000;

	// SEQID 1 - Write enable
	QuadSPI -> LUT[4]= 0x406;
	QuadSPI -> LUT[5]= 0x0000;

	// SEQID 2 - Read Status
	QuadSPI -> LUT[8]= 0x1c080405;
	QuadSPI -> LUT[9]= 0x00000C04;

	// SEQID 3 - Write Status
	QuadSPI -> LUT[12]= 0x20010401;
	QuadSPI -> LUT[13]= 0x00000000;

	// SEQID 4 - Sector Erase
	QuadSPI -> LUT[16] = 0x08180420;
	QuadSPI -> LUT[17] = 0x00000000;

	// SEQID 5 - Page Program
	QuadSPI -> LUT[20] = 0x08180402;
	QuadSPI -> LUT[21] = 0x00002080;

	// SEQID 6 - Single line read
	QuadSPI -> LUT[24] = 0x08180403; /* Normal read command, 24-bit address */
	QuadSPI -> LUT[25] = 0x1C800C08; /* 6 dummy cycle wait. 128 bytes read */
	QuadSPI -> LUT[26] = 0x00002400; /* jump to CS command */
	QuadSPI -> LUT[27] = 0x00000000;

	// SEQID 7 - Quad line read
	QuadSPI -> LUT[28] = 0x0A1804EB; /* Quad I/0 read command, 24-bit address */
	QuadSPI -> LUT[29] = 0x1E800E06; /* 6 dummy cycle wait on 4 lines, 128 bytes read on 4 lines */
	QuadSPI -> LUT[30] = 0x00002400; /* jump to CS command */
	QuadSPI -> LUT[31] = 0x00000000;

	//Lock the LUT
	QuadSPI -> LUTKEY = 0x5AF05AF0;
	QuadSPI -> LCKCR = 0x1;	//LOCK the LUT
	while(((QuadSPI -> LCKCR)&QuadSPI_LCKCR_LOCK_MASK)>>QuadSPI_LCKCR_LOCK_SHIFT == 0);
}


void quadspi_erase_sector(uint32_t address)
{
	//write enable
	quadspi_write_enable();
	QuadSPI->SFAR = address;

	//	send erase command
	launch_loot(SECTOR_ERASE);

	QuadSPI->IPCR;
	quadspi_wait_while_flash_busy();
}


void quadspi_set_lut(uint32_t index, uint32_t value)  //gaurav
{
	/* Unlock the LUT */
	do{
		QuadSPI->LUTKEY = 0x5AF05AF0;
		QuadSPI->LCKCR = 0x2;	//UNLOCK the LUT
	}
	while(((QuadSPI -> LCKCR)&QuadSPI_LCKCR_UNLOCK_MASK)>>QuadSPI_LCKCR_UNLOCK_SHIFT == 0); /*Wait for LUT to be unlocked */

	QuadSPI->LUT[index] = value;

	/* Lock the LUT */
	QuadSPI->LUTKEY = 0x5AF05AF0;
	QuadSPI->LCKCR = 0x1;	//LOCK the LUT
	while(((QuadSPI -> LCKCR)&QuadSPI_LCKCR_LOCK_MASK)>>QuadSPI_LCKCR_LOCK_SHIFT == 0); /* Wait for LUT to be locked */
}



void quadspi_read_write()
{
	uint32_t i,read_counter=0;
	uint32_t * ptr_bin_image = (uint32_t *)0x100000;

	quadspi_quad_en(); /* Enable QuadMode */

	/* Erase required sector. Each sector is 4Kb long */
	quadspi_erase_sector(0x68000000);

	single_quadspi_program((uint32_t)ptr_bin_image,FLASH_A_BASE_ADDR,4096); /*Store Binary Image into flash. Program first sector */

	quad_quadspi_read(FLASH_A_BASE_ADDR,qspi_dest_cpu,4096); /* Reading full sector (4 K bytes) through the commands interface */

	for(i = 0; i<128; i++){ /* read 4096 bytes. 4096 / 32 bytes array = 128 */
		if(qspi_dest_cpu[i]==ptr_bin_image[i]){
			read_counter++; /* Correct reading increment counter */
		}
		else{
			while(1){
				/* error trap */
			}
		}
	}

	quadspi_AHB_enable();

	/* change for AHB reading endianess */
	QuadSPI -> MCR &= ~(QuadSPI_MCR_END_CFG_MASK);
	QuadSPI -> MCR |= 0x08;

} /* QSPI_setup */


void quadspi_wait_while_flash_busy()
{
	uint32_t status_value = 0;
	do{
		launch_loot(READ_STATUS);
		status_value = QuadSPI->RBDR[0];
		status_value = ((status_value>>24)&0x00000001); /* Poll for the external memory busy flash */
		QuadSPI->MCR = QuadSPI_MCR_CLR_RXF_MASK;	 /* Clean receive flag */
		QuadSPI->FR = 0x10000; /* read complete */
	}while( status_value == 1);
}


void single_quadspi_program(uint32_t src, uint32_t base, uint32_t size)
{
	uint32_t i,m,j,page_iterations,remain_bytes;
	uint32_t *data = (uint32_t *)src;

	if(size>FLASH_PGSZ){
		remain_bytes = FLASH_PGSZ;
		page_iterations = (size/FLASH_PGSZ);
	}
	else{
		remain_bytes = size;
		page_iterations = 1;
	}
	for(i=0;i<page_iterations;i++){
		quadspi_write_enable();
		QuadSPI -> MCR |= QuadSPI_MCR_CLR_TXF_MASK;
		QuadSPI -> FR = 0x08000000;
		m = remain_bytes/4; /* TBDR buffer is 4 bytes long */
		for(j = 0; j<m; j++)
		{
			QuadSPI->TBDR = *data++;
		}

		//set address
		QuadSPI->SFAR = base;

		/* Launch Page Program commmand */
		launch_loot(PAGE_PROGRAM);

		quadspi_wait_while_flash_busy(); 		//check status, wait to be done
		base += FLASH_PGSZ;
		if(size > FLASH_PGSZ){
			remain_bytes = FLASH_PGSZ;
			size = size - FLASH_PGSZ;
		}
		else{
			remain_bytes = size;
		}
	}
}


void single_quadspi_read(uint32_t address, uint32_t *dest, uint32_t size)
{
	uint16_t i,j;

	QuadSPI->SFAR = address;
	size = size/32;
	for(i = 0; i<size; i++)
	{
		QuadSPI->MCR |= QuadSPI_MCR_CLR_RXF_MASK;
		QuadSPI->FR = 0x10000;

		/* Launch single read command */
		launch_loot(SINGLE_READ);
		while(((QuadSPI->RBSR & QuadSPI_RBSR_RDBFL_MASK)>>QuadSPI_RBSR_RDBFL_SHIFT)!=32);
		//RX buffer size is 32 words
		for(j = 0; j<32; j++)
		{
			*dest++ = QuadSPI->RBDR[j];
		}
		QuadSPI->SFAR = QuadSPI->SFAR + (32*4);
	}
	QuadSPI->MCR |= QuadSPI_MCR_CLR_RXF_MASK;
}


void quad_quadspi_read(unsigned long address, unsigned long *dest, unsigned long size)
{
	int i,j;

	QuadSPI->SFAR = address;
	size = size/32;
	for(i = 0; i<size; i++)
	{
		QuadSPI->MCR |= QuadSPI_MCR_CLR_RXF_MASK;
		QuadSPI->FR = 0x10000;

		/* launch quad read command */
		launch_loot(QUAD_READ);

		while(((QuadSPI->RBSR & QuadSPI_RBSR_RDBFL_MASK)>>QuadSPI_RBSR_RDBFL_SHIFT)!=32);
		//RX buffer size is 32 words
		for(j = 0; j<32; j++)
		{
			*dest++ = QuadSPI->RBDR[j];
		}
		QuadSPI->SFAR = QuadSPI->SFAR + (32*4);
	}
	QuadSPI->MCR |= QuadSPI_MCR_CLR_RXF_MASK;
}

void quadspi_quad_en(void){
	uint8_t j;

	quadspi_write_enable();

	for(j = 0; j<32; j++)   QuadSPI->TBDR = 0x42424242 ;   /* At least 128 bytes should be there in TX FIFO to start TX*/

	/* Launch write status command */
	launch_loot(WRITE_STATUS);

	quadspi_wait_while_flash_busy();

}

void quadspi_write_enable(void){
	launch_loot(WRITE_ENABLE);
}

void launch_loot(uint8_t loot_index){
	QuadSPI->IPCR = (loot_index>>2) << 24;
	while(QuadSPI->SR & QuadSPI_SR_BUSY_MASK);
}

void quadspi_AHB_enable(void){

	quadspi_set_lut(0,0x0A1804EB); /* SET LUT 0 for quadline read */
	quadspi_set_lut(1,0x1E800E06);
	quadspi_set_lut(2,0x00002400);
	quadspi_set_lut(3,0x00000000);

}


