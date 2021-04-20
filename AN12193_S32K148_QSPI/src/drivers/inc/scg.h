/*
 * template.h
 *
 *  Rev:
 *  Author:
 *  Date:
 */

#ifndef SCG_H_
#define SCG_H_

/* Only include these header files */
#include "error_codes.h"
#include "scg_config.h"

/* Possible clock sources */
typedef enum {
    eCoreClock = 0,
    eBusClock,
    eFlashClock,
    eSIRClk,
    eFIRClk,
    eSysOsc,
    eSysPLL,
    eSysPLL_div1,
    eSysPLL_div2,
    eFIRC_div1,
    eFIRC_div2,
    eSIRC_div1,
    eSIRC_div2,
    eSysOsc_div1,
    eSysOsc_div2
}eClock;

/**************************************************************************//*!
 *
 * @name    SCG_init
 *
 * @brief   Initialize SCG module. Set the core, bus and flash clocks according
 *          configuration structure. If core frequency is set to be less or equal
 *          than 4MHz, then MCU is moved to VLPR mode, if frequency is below or
 *          equal 80MHz, then MCU is moved to RUN mode, and if frequency is
 *          higher than 80MHz, MCU is moved to HSRUN mode.
 *          The function check if core clock can be fed by SIRC first, if not,
 *          FIRC is checked, if not, then SPLL will be used. Function calculates
 *          the closest frequency (From highest possible to lowest) to meet the
 *          desired frequency.
 *
 * @param   config_ptr: Configuration structure for this driver.
 *
 * @return  eNoError: If initialization was done without problems.
 *          eInvalidParameter: If any argument is invalid.
 *          eNullPointer: If pointer to configuration structure is NULL
 *          eModuleIsBusy: If clock has been already initialized.
 *****************************************************************************/
eErrorCodes SCG_init(SCG_config_t * const config_ptr);

/**************************************************************************//*!
 *
 * @name    SCG_get_core_clocks
 *
 * @brief   Get the frequency for selected clock source.
 *
 * @param   eClock src: Selected source clock.
 *
 * @return uint32_t: clock frequency, 0 if disabled.
 *****************************************************************************/
uint32_t SCG_get_clock_freq(eClock src);

/**************************************************************************//*!
 *
 * @name    SCG_set_clock_freq
 *
 * @brief   Set the frequency for selected clock source. For Core clock frequency,
 *          as flash and bus clocks depend on core frequency, any change in
 *          core clock will impact the value for flash and bus, setting those to
 *          maximum value according to the current SCG mode: VLPR, RUN or HSRUN.
 *
 * @param   eClock src: Selected source clock.
 *          uint32_t freq: Desired frequency value
 *
 * @return  eNoError: If initialization was done without problems.
 *          eInvalidParameter: If any argument is invalid.
 *****************************************************************************/
eErrorCodes SCG_set_clock_freq(eClock src, uint32_t freq);


#endif /* TEMPLATE_H_ */
