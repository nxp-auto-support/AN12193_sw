/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


#include "scg.h" /* include peripheral declarations S32K144 */
#include "S32K148.h"
#include "QSPI.h"
extern SCG_config_t configStruct;

/* Flash operations/locations */
#define SIZE_DATA 		(128)			/* How many words (x4 bytes) to be programmed to flash */

uint32_t * test_ptr;
uint32_t test_value;
typedef void (*ptr_fnc)(void);


void PORT_init(void){
	PCC-> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORTC */
	PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORTD */
	PORTC->PCR[2] =  PORT_PCR_MUX(7);  /* Port C2:  MUX = QSPI IO3 */
	PORTC->PCR[3] =  PORT_PCR_MUX(6);  /* Port C3:  MUX = QSPI CS */
	PORTD->PCR[10] =  PORT_PCR_MUX(7);  /* Port D10:  MUX = QSPI SCK */
	PORTD->PCR[12] =  PORT_PCR_MUX(7);  /* Port D12:  MUX = QSPI IO2 */
	PORTD->PCR[7] =  PORT_PCR_MUX(7);  /* Port C2:  MUX = QSPI IO1 */
	PORTD->PCR[11] =  PORT_PCR_MUX(7);  /* Port D11:  MUX = QSPI IO0 */
}

int main(void)
{

    eErrorCodes ret;
	ptr_fnc hello_main = (ptr_fnc) (0x68000194+1);

	/* Initialize SCG module */
    ret = SCG_init(&configStruct);
    if (eNoError != ret)
    {
        /* Error trap */
        while (1)
        {
            __asm("nop");
        }
    }

    SCG_set_clock_freq(eSysPLL_div1,80000000);
    SCG_set_clock_freq(eSysPLL_div2,80000000);
    SCG_set_clock_freq(eFIRC_div2,24000000);

    QSPI_setup(); /* Initialize QuadSPI */
	PORT_init(); /* Initialize ports */
    quadspi_read_write(); /* Write binary image to external flash */

    for(;;) {
    	hello_main(); /* Jump to QuadSPI loaded application */
    }
	return 0;
}
