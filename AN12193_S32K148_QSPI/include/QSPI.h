/*
 * QSPI.h
 *
 *  Created on: Mar 11, 2017
 *      Author: B55840
 */

#ifndef QSPI_H_
#define QSPI_H_

#include "S32K148.h"

#define FLASH_A_BASE_ADDR	0x68000000
#define FLASH_B_BASE_ADDR	0x6c000000


void QSPI_setup(void);
void single_quadspi_program(uint32_t src, uint32_t base, uint32_t size);
void single_quadspi_read(uint32_t address, uint32_t *dest, uint32_t size);
void quad_quadspi_read(uint32_t address, uint32_t *dest, uint32_t size);
void quadspi_read_write();
void quadspi_erase_sector(uint32_t address);
void quadspi_quad_en(void);
void quadspi_AHB_enable(void);

/////////////////required defines to compile //////////////////////////////////
#define BURST_SIZE 0x80 //max 128bytes!
#define FLASH_PGSZ	(128)
#define FLASH_DMA_PGSZ (512)

/* LUT sequences */
#define WRITE_ENABLE	4
#define READ_STATUS		8
#define WRITE_STATUS	12
#define SECTOR_ERASE	16
#define PAGE_PROGRAM	20
#define SINGLE_READ		24
#define QUAD_READ		28

/* QUADSPI Instructions */
#define CMD 1
#define ADDR 2
#define DUMMY 3
#define MODE1 4
#define MODE2 5
#define MODE4 6
#define READ 7
#define WRITE 8
#define JMP_ON_CS 9
#define ADDR_DDR 10
#define MODE_DDR 11
#define MODE2_DDR 12
#define MODE4_DDR 13
#define READ_DDR 14
#define WRITE_DDR 15
#define DATA_LEARN 16
#define CMD_DDR 17
#define CADDR 18
#define CADDR_DDR 19
#define STOP 0

#define QSPI_LUT(CMD1,PAD1,OP1,CMD0,PAD0,OP0)	((((CMD1)&0x3f)<<26)|(((PAD1)&3)<<24)|(((OP1)&0xff)<<16)|(((CMD0)&0x3f)<<10)|(((PAD0)&3)<<8)|((OP0)&0xff))


#endif /* QSPI_H_ */
