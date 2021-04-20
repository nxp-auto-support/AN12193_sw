/*
 * template.c
 *
 *  Created on: Feb 10, 2017
 *      Author: B50982
 */

/* Includes */
#include "scg.h"
#include "system.h"

/* MACRO Definitions */
#define CALCULATE_VCO_FREQ(prediv,mult)                 ((((float)OSC_VALUE) / (prediv + 1)) * (mult + 16))

/* Macro to validate all possible frequencies after dividing them by DIVCORE */
#define VALIDATE_SRC_RANGES(src_freq, desired_freq)     ((src_freq == (desired_freq)) ||         \
                                                         ((src_freq/2) == (desired_freq)) ||     \
                                                         ((src_freq/3) == (desired_freq)) ||     \
                                                         ((src_freq/4) == (desired_freq)) ||     \
                                                         ((src_freq/5) == (desired_freq)) ||     \
                                                         ((src_freq/6) == (desired_freq)) ||     \
                                                         ((src_freq/7) == (desired_freq)) ||     \
                                                         ((src_freq/8) == (desired_freq)) ||     \
                                                         ((src_freq/9) == (desired_freq)) ||     \
                                                         ((src_freq/10) == (desired_freq)) ||    \
                                                         ((src_freq/11) == (desired_freq)) ||    \
                                                         ((src_freq/12) == (desired_freq)) ||    \
                                                         ((src_freq/13) == (desired_freq)) ||    \
                                                         ((src_freq/14) == (desired_freq)) ||    \
                                                         ((src_freq/15) == (desired_freq)) ||    \
                                                         ((src_freq/16) == (desired_freq)))
/* Macro to get PREDIV and MULT values for VCO calculation */
#define SPLL_PREDIV_VAL()                               ((SCG->SPLLCFG & SCG_SPLLCFG_PREDIV_MASK) >> SCG_SPLLCFG_PREDIV_SHIFT)
#define SPLL_MULT_VAL()                                 ((SCG->SPLLCFG & SCG_SPLLCFG_MULT_MASK) >> SCG_SPLLCFG_MULT_SHIFT)

/* Macro to get/set DIV1 and DIV2 values for asynchronous sources */
#define GET_ASYNC_CLK_DIV2(reg)                         ((reg & (0x700)) >> 8)
#define GET_ASYNC_CLK_DIV1(reg)                         ((reg & (0x7)) >> 0)

#define SET_ASYNC_CLK_DIV2(val)                         ((val & (0x7)) << 8)
#define SET_ASYNC_CLK_DIV1(val)                         ((val & (0x7)) << 0)

/* Maximum clock rates for Core, Bus and Flash in VLPR mode */
#define VLPR_MAXIMUM_CORE_FREQ                          4000000
#define VLPR_MAXIMUM_BUS_FREQ                           VLPR_MAXIMUM_CORE_FREQ
#define VLPR_MAXIMUM_FLASH_FREQ                         (VLPR_MAXIMUM_CORE_FREQ >> 2)

/* Maximum clock rates for Core, Bus and Flash in RUN mode */
#define RUN_MAXIMUM_CORE_FREQ                           80000000
#define RUN_MAXIMUM_BUS_FREQ_FIRC                       48000000
#define RUN_MAXIMUM_BUS_FREQ_SPLL                       (RUN_MAXIMUM_CORE_FREQ >> 1) /* 80MHz / 2 */
#define RUN_MAXIMUM_FLASH_FREQ                          26670000

/* Maximum clock rates for Core, Bus and Flash in HSRUN mode */
#define HSRUN_MAXIMUM_CORE_FREQ                         112000000
#define HSRUN_MAXIMUM_BUS_FREQ                          (HSRUN_MAXIMUM_CORE_FREQ >> 1) /* 112 MHz / 2 */
#define HSRUN_MAXIMUM_FLASH_FREQ                        (HSRUN_MAXIMUM_CORE_FREQ >> 2) /* 112 MHz / 4 */

typedef enum {
    SOSC = 1,
    SIRC,
    FIRC,
    SPLL = 6
}eSystemClockSource;

typedef enum {
    eVLPR,
    eRun,
    eHSRun
}eSCGModes;

/* Helper Functions */
static eErrorCodes SCG_spll_config(uint32_t *const freq);
static void SCG_sirc_init(void);
static void SCG_firc_init(void);
static void SCG_sosc_init(void);
static eErrorCodes SCG_spll_init(uint32_t * const freq);
static eErrorCodes SCG_set_core_dividers(eSystemClockSource src, uint32_t coreFreq,
                                        uint32_t busFreq, uint32_t flashFreq);
static int8_t SCG_get_best_divider (uint32_t desiredFreq, uint32_t srcFreq);
static int8_t SCG_get_best_async_dividers (uint32_t desiredFreq, uint32_t srcFreq);
static uint32_t SCG_get_firc_freq(void);
static uint32_t SCG_get_sirc_freq(void);
static uint32_t SCG_get_sosc_freq(void);
static uint32_t SCG_get_spll_freq(void);
static uint32_t SCG_get_core_freq(void);
static uint32_t SCG_get_bus_freq(void);
static uint32_t SCG_get_flash_freq(void);

eErrorCodes SCG_init(SCG_config_t *const config_ptr)
{
    eErrorCodes ret = eNoError;
    if (NULL != config_ptr)
    {
        /* Validate all possible frequencies that can be set for SIRC */
        if (VALIDATE_SRC_RANGES(SIRC_VALUE, config_ptr->coreFreq))
        {
            /* Enable SIRC */
            SCG_sirc_init();
            ret = SCG_set_core_dividers(SIRC, config_ptr->coreFreq, config_ptr->busFreq, config_ptr->flashFreq);
        }
        /* Validate all possible frequencies that can be set for FIRC */
        else if (VALIDATE_SRC_RANGES(FIRC_VALUE, config_ptr->coreFreq))
        {
            /* Enable FIRC */
            SCG_firc_init();
            ret = SCG_set_core_dividers(FIRC, config_ptr->coreFreq, config_ptr->busFreq, config_ptr->flashFreq);
        }
        /* if SIRC and FIRC cannot be used, then, try to use the SPLL */
        else
        {
            uint32_t freq = config_ptr->coreFreq;
            SCG_sosc_init();
            ret = SCG_spll_init(&freq);
            if (eNoError == ret)
            {
                ret = SCG_set_core_dividers(SPLL, freq, config_ptr->busFreq, config_ptr->flashFreq);
            }
        }
    }
    else
    {
        ret = eNullPointer;
    }
    return ret;
}

uint32_t SCG_get_clock_freq(eClock src)
{
    uint32_t freq = 0, div = 0;
    switch (src)
    {
    case eCoreClock:
        freq = SCG_get_core_freq();
        break;
    case eBusClock:
        freq = SCG_get_bus_freq();
        break;
    case eFlashClock:
        freq = SCG_get_flash_freq();
        break;
    case eSIRClk:
        freq = SCG_get_sirc_freq();
        break;
    case eFIRClk:
        freq = SCG_get_firc_freq();
        break;
    case eSysOsc:
        freq = SCG_get_sosc_freq();
        break;
    case eSysPLL:
        freq = SCG_get_spll_freq();
        break;
    case eSysPLL_div1:
        {
            freq = SCG_get_spll_freq();
            div = GET_ASYNC_CLK_DIV1(SCG->SPLLDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    case eSysPLL_div2:
        {
            freq = SCG_get_spll_freq();
            div = GET_ASYNC_CLK_DIV2(SCG->SPLLDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    case eFIRC_div1:
        {
            freq = SCG_get_firc_freq();
            div = GET_ASYNC_CLK_DIV1(SCG->FIRCDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    case eFIRC_div2:
        {
            freq = SCG_get_firc_freq();
            div = GET_ASYNC_CLK_DIV2(SCG->FIRCDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    case eSIRC_div1:
        {
            freq = SCG_get_sirc_freq();
            div = GET_ASYNC_CLK_DIV1(SCG->SIRCDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    case eSIRC_div2:
        {
            freq = SCG_get_sirc_freq();
            div = GET_ASYNC_CLK_DIV2(SCG->SIRCDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    case eSysOsc_div1:
        {
            freq = SCG_get_sosc_freq();
            div = GET_ASYNC_CLK_DIV1(SCG->SOSCDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    case eSysOsc_div2:
        {
            freq = SCG_get_sosc_freq();
            div = GET_ASYNC_CLK_DIV2(SCG->SOSCDIV);
            /* Output disabled ? */
            if (0 == div)
            {
                freq = 0;
            }
            else
            {
                div = (1 << (div - 1));
                freq /= div;
            }
        }
        break;
    default:
        break;
    }
    return freq;
}

eErrorCodes SCG_set_clock_freq(eClock src, uint32_t freq)
{
    eErrorCodes ret = eNoError;
    switch (src)
    {
    case eSIRClk:
        SCG_sirc_init();
        break;
    case eFIRClk:
        SCG_firc_init();
        break;
    case eSysOsc:
        SCG_sosc_init();
        break;
    case eSysPLL:
        {
            uint32_t temp = freq;
            ret = SCG_spll_init(&temp);
        }
        break;
    case eSysPLL_div1:
        {
            /* Disable output */
            SCG->SPLLDIV &= ~(SCG_SPLLDIV_SPLLDIV1_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_spll_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->SPLLDIV |= SCG_SPLLDIV_SPLLDIV1(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eSysPLL_div2:
        {
            /* Disable output */
            SCG->SPLLDIV &= ~(SCG_SPLLDIV_SPLLDIV2_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_spll_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->SPLLDIV |= SCG_SPLLDIV_SPLLDIV2(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eFIRC_div1:
        {
            /* Disable output */
            SCG->FIRCDIV &= ~(SCG_FIRCDIV_FIRCDIV1_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_firc_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->FIRCDIV |= SCG_FIRCDIV_FIRCDIV1(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eFIRC_div2:
        {
            /* Disable output */
            SCG->FIRCDIV &= ~(SCG_FIRCDIV_FIRCDIV2_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_firc_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->FIRCDIV |= SCG_FIRCDIV_FIRCDIV2(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eSIRC_div1:
        {
            /* Disable output */
            SCG->SIRCDIV &= ~(SCG_SIRCDIV_SIRCDIV1_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_sirc_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->SIRCDIV |= SCG_SIRCDIV_SIRCDIV1(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eSIRC_div2:
        {
            /* Disable output */
            SCG->SIRCDIV &= ~(SCG_SIRCDIV_SIRCDIV2_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_sirc_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->SIRCDIV |= SCG_SIRCDIV_SIRCDIV2(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eSysOsc_div1:
        {
            /* Disable output */
            SCG->SOSCDIV &= ~(SCG_SOSCDIV_SOSCDIV1_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_sosc_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->SOSCDIV |= SCG_SOSCDIV_SOSCDIV1(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eSysOsc_div2:
        {
            /* Disable output */
            SCG->SOSCDIV &= ~(SCG_SOSCDIV_SOSCDIV2_MASK);
            if (0 != freq)
            {
                int8_t div = SCG_get_best_async_dividers(freq, SCG_get_sosc_freq());
                if ((div > 0) && (div < 8))
                {
                    SCG->SOSCDIV |= SCG_SOSCDIV_SOSCDIV2(div);
                }
                else
                {
                    ret = eInvalidParameter;
                }
            }
        }
        break;
    case eCoreClock:
        if (0 != freq)
        {
            /* Validate all possible frequencies that can be set for SIRC */
            if (VALIDATE_SRC_RANGES(SIRC_VALUE, freq))
            {
                /* Enable SIRC */
                SCG_sirc_init();
                ret = SCG_set_core_dividers(SIRC, freq, MAXIMUM_AVAILABLE_FREQ, MAXIMUM_AVAILABLE_FREQ);
            }
            /* Validate all possible frequencies that can be set for FIRC */
            else if (VALIDATE_SRC_RANGES(FIRC_VALUE, freq))
            {
                /* Enable FIRC */
                SCG_firc_init();
                ret = SCG_set_core_dividers(FIRC, freq, MAXIMUM_AVAILABLE_FREQ, MAXIMUM_AVAILABLE_FREQ);
            }
            /* if SIRC and FIRC cannot be used, then, try to use the SPLL */
            else
            {
                uint32_t tempfreq = freq;
                SCG_sosc_init();
                ret = SCG_spll_init(&tempfreq);
                if (eNoError == ret)
                {
                    ret = SCG_set_core_dividers(SPLL, tempfreq, MAXIMUM_AVAILABLE_FREQ, MAXIMUM_AVAILABLE_FREQ);
                }
            }
        }
        break;
    case eBusClock:
        ret = SCG_set_core_dividers((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT,
                    SCG_get_core_freq(), freq, MAXIMUM_AVAILABLE_FREQ);
        break;
    case eFlashClock:
        ret = SCG_set_core_dividers((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT,
                    SCG_get_core_freq(), SCG_get_bus_freq(), freq);
        break;
    default:
        break;
    }
    return ret;
}

/**************************************************************************//*!
 *
 * @name    SCG_sirc_init
 *
 * @brief   Function used to initialize SIRC module
 *
 * @param   none.
 *
 * @return  none.
 *****************************************************************************/
static void SCG_sirc_init(void)
{
    /* Check if current source is not enabled and valid yet */
    if ((0 == (SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK)) &&
                (0 == (SCG->SIRCCSR & SCG_SIRCCSR_SIRCSEL_MASK)))
    {
        /* Unlock the register */
        SCG->SIRCCSR &= ~SCG_SIRCCSR_LK_MASK;
        /* Configure SIRC */
        SCG->SIRCCSR |= SCG_SIRCCSR_SIRCLPEN_MASK | /* Enable SIRC in VLP mode */
                        SCG_SIRCCSR_SIRCSTEN_MASK | /* Enable SIRC in STOP mode */
                        SCG_SIRCCSR_SIRCEN_MASK;    /* Enable SIRC */
        /* Wait until SIRC is enabled */
        while (0 == (SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK)) { };
        /* Lock the register */
        SCG->SIRCCSR |= SCG_SIRCCSR_LK_MASK;
    }
}

/**************************************************************************//*!
 *
 * @name    SCG_firc_init
 *
 * @brief   Function used to initialize FIRC module
 *
 * @param   none.
 *
 * @return  none.
 *****************************************************************************/
static void SCG_firc_init(void)
{
    /* Check if current source is not enabled and valid yet */
    if ((0 == (SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK)) &&
                (0 == (SCG->FIRCCSR & SCG_FIRCCSR_FIRCSEL_MASK)))
    {
        /* Unlock the register */
        SCG->FIRCCSR &= ~SCG_FIRCCSR_LK_MASK;
        /* Enable FIRC */
        SCG->FIRCCSR |= SCG_FIRCCSR_FIRCEN_MASK;
        /* Wait until FIRC is enabled */
        while (0 == (SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK)) { };
        /* Lock the register */
        SCG->FIRCCSR |= SCG_FIRCCSR_LK_MASK;
    }
}

/**************************************************************************//*!
 *
 * @name    SCG_sosc_init
 *
 * @brief   Function used to initialize System OSC (SOSC) module
 *
 * @param   none.
 *
 * @return  none.
 *****************************************************************************/
static void SCG_sosc_init(void)
{
    /* Check if current source is not enabled and valid yet */
    if ((0 == (SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK)) &&
            (0 == (SCG->SOSCCSR & SCG_SOSCCSR_SOSCSEL_MASK)))
    {
        uint32_t sosccfg_mask =
#if (OSC_VALUE < 8000000)
        SCG_SOSCCFG_RANGE(0b10);
#elif (OSC_VALUE <= 40000000)
        SCG_SOSCCFG_RANGE(0b11);
#else
        0;
#endif
        SCG->SOSCCFG =  SCG_SOSCCFG_EREFS_MASK |    /* Selects the output of the OSC logic (crystal is used) */
                        sosccfg_mask;               /* Range is very high frequency crystal */
        /* Unlock the register */
        SCG->SOSCCSR &= ~SCG_SOSCCSR_LK_MASK;
        /* Enable System Oscillator */
        SCG->SOSCCSR |= SCG_SOSCCSR_SOSCEN_MASK;
        /* Wait until SOSC is enabled */
        while (0 == (SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK)) { };
        /* Lock the register */
        SCG->SOSCCSR |= SCG_SOSCCSR_LK_MASK;
    }
}

/**************************************************************************//*!
 *
 * @name    SCG_spll_init
 *
 * @brief   Function used to initialize System PLL (SPLL) module. It calculates
 *          PREDIV and MULT values according desired frequency and it returns
 *          SPLL's output frequency in the [IN/OUT] frequency pointer parameter.
 *
 * @param   uint32_t *frequency [IN/OUT]: Pointer that indicates desired frequency
 *                                        for SPLL. If frequency cannot be set,
 *                                        then, SPLL will be adjusted to closest
 *                                        frequency and pointer will save this frequency.
 *
 * @return  eNoError: SPLL initialized correctly.
 *          eModuleIsBusy: Core is using SPLL and it cannot be disabled.
 *          eNullPointer: NULL frequency pointer.
 *****************************************************************************/
static eErrorCodes SCG_spll_init(uint32_t * const frequency)
{
    eErrorCodes ret = eNoError;
    if (NULL != frequency)
    {
        ret = SCG_spll_config(frequency);
        /* Check if current source is not enabled and valid yet */
        if ((0 == (SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)) &&
                    (0 == (SCG->SPLLCSR & SCG_SPLLCSR_SPLLSEL_MASK)))
        {
            /* Unlock the register */
            SCG->SPLLCSR &= ~SCG_SPLLCSR_LK_MASK;
            /* Enable System PLL */
            SCG->SPLLCSR |= SCG_SPLLCSR_SPLLEN_MASK;
            /* Wait until SPLL is enabled */
            while (0 == (SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)) { };
            /* Lock the register */
            SCG->SPLLCSR |= SCG_SPLLCSR_LK_MASK;
        }
        else
        {
            ret = eModuleIsBusy;
        }
    }
    else
    {
        ret = eNullPointer;
    }
    return ret;
}

/**************************************************************************//*!
 *
 * @name    SCG_spll_config
 *
 * @brief   Function used to calculate PREDIV and MULT values for SPLL.
 *          It calculates PREDIV and MULT values according desired frequency
 *          and it returns SPLL's output frequency in the [IN/OUT] frequency
 *          pointer parameter. It also consider possible DIV core values for
 *          SPLL and calculates frequency from highest possible to lowest.
 *
 * @param   uint32_t *frequency [IN/OUT]: Pointer that indicates desired frequency
 *                                        for SPLL. If frequency cannot be set,
 *                                        then, SPLL will be adjusted to closest
 *                                        frequency and pointer will save this frequency.
 *
 * @return  eNoError: SPLL initialized correctly.
 *          eNullPointer: NULL frequency pointer.
 *****************************************************************************/

static eErrorCodes SCG_spll_config(uint32_t *const freq) {
    eErrorCodes ret = eNoError;
    int8_t bestMult = 0, bestDivider = 0, bestPrediv = 0, done = false;
    float calculated_freq = 0;
    float minError, difference;
    register int8_t mult = 0, divider = 0, prediv = 0;

    if (NULL != freq)
    {
        /* Set maximum possible error */
        minError = *freq;
        /* flag to signal when calculation is done */
        done = 0;
        /*
         * PREDIV: The resulting frequency must be in the range of 8 MHz to 32 MHz.
         * As SOSC is used (8MHz), only 0 value for PREDIV can be used
         */
        for (prediv = 0; (prediv >= 0) && (false == done); prediv++)
        {
            /* Iterate between all MULT available values (From highest to lowest) */
            for (mult = 31; (mult >= 0)  && (false == done); mult--) {
                /* Consider DIVCORE values for core frequency */
                for (divider = 1; (divider < 16) && (false == done); divider++)
                {
                    /* Calculate SPLL frequency */
                    calculated_freq = CALCULATE_VCO_FREQ(prediv,mult);
                    if ((calculated_freq >= 180000000) && (calculated_freq <= 360000000))
                    {
                        calculated_freq = (CALCULATE_VCO_FREQ(prediv,mult) / 2) / divider;
                        /* get the difference between desired and calculated frequency */
                        difference = (calculated_freq > *freq ? (calculated_freq - *freq) : (*freq - calculated_freq));
                        /* If difference is less than current minimum error, save MULT, PREDIV and DIVCORE values */
                        if (difference < minError)
                        {
                            /* Set the minimum difference */
                            minError = difference;
                            /* Save MULT value */
                            bestMult = mult;
                            /* Save best divider */
                            bestDivider = divider;
                            /* Save prediv value */
                            bestPrediv = prediv;
                            /* If difference is zero, then these are the exact
                             * factors for desired frequency */
                            if (0 == difference)
                            {
                                /* calculation is over */
                                done = true;
                            }
                        }
                    }

                }
            }
        }

        /* Set best MULT and PREDIV (0) factors */
        SCG->SPLLCFG =  SCG_SPLLCFG_MULT(bestMult) | SCG_SPLLCFG_PREDIV(bestPrediv);
        /* Calculate closest frequency and saved in pointer parameter */
        *freq = ((((float)OSC_VALUE / (bestPrediv+1)) * (bestMult + 16)) / 2) / (bestDivider);
    }
    else
    {
        ret = eNullPointer;
    }
    return ret;
}


/**************************************************************************//*!
 *
 * @name    SCG_set_core_dividers
 *
 * @brief   Function used to set the proper DIVCORE, DIVBUS and DIVSLOW values.
 *          If DIVBUS and DIVSLOW cannot be set to a value to match the desired
 *          frequency for Bus and flash respectively, function will set the best
 *          divider to get the closest frequency.
 *          If core frequency is less than 4MHz, then transition to VLPR is made,
 *          if core frequency is greater than 80 MHz, then transition to HSRUN is
 *          made.
 *
 * @param   eClockSources src: Source that will provide the clock for core.
 *          uint32_t      coreFreq: Desired core clock.
 *          uint32_t      busFreq: Desired bus clock.
 *          uint32_t      flashFreq: Desired flash clock.
 *
 * @return  eNoError: SPLL initialized correctly.
 *          eInvalidParameter: Invalid parameters.
 *****************************************************************************/
static eErrorCodes SCG_set_core_dividers(eSystemClockSource src, uint32_t coreFreq,
                                        uint32_t busFreq, uint32_t flashFreq)
{
    eErrorCodes ret = eNoError;
    int8_t coreDivider = 0, busDivider = 0, flashDivider = 0;
    uint32_t srcFreq = 0;
    volatile uint32_t *scg_reg_ptr = NULL;
    eSCGModes mode = eRun;

    /* VLPR mode */
    if (VLPR_MAXIMUM_CORE_FREQ >= coreFreq)
    {
        /* Only SIRC can be used in VLPR mode, check if SIRC is available */
        if ((SIRC == src) &&
                (((SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK) >> SCG_SIRCCSR_SIRCVLD_SHIFT) == 1))
        {
            /* Calculate best DIVCORE value */
            coreDivider = SCG_get_best_divider(coreFreq, SIRC_VALUE);
            if ((coreDivider >= 0) && (coreDivider < 16))
            {
                scg_reg_ptr = &(SCG->VCCR);
                mode = eVLPR;
            }
            else
            {
                scg_reg_ptr = NULL;
            }
            /* Calculate best DIVBUS value */
            busDivider = SCG_get_best_divider((busFreq <= VLPR_MAXIMUM_BUS_FREQ) ?
                                                busFreq : VLPR_MAXIMUM_BUS_FREQ, SIRC_VALUE);
            if ((busDivider < 0) || (busDivider >= 16))
            {
                ret = eInvalidParameter;
            }
            /* Calculate best DIVSLOW value */
            flashDivider = SCG_get_best_divider((flashFreq <= VLPR_MAXIMUM_FLASH_FREQ) ?
                                                    flashFreq : VLPR_MAXIMUM_FLASH_FREQ, SIRC_VALUE);
            if ((flashDivider < 0) || (flashDivider >= 16))
            {
                ret = eInvalidParameter;
            }
        }
        else
        {
            ret = eInvalidParameter;
        }
    }
    /* Normal Run mode */
    else if (RUN_MAXIMUM_CORE_FREQ >= coreFreq)
    {
        uint32_t maximumBusFreq = 0;
        /* clear source frequency */
        srcFreq &= 0;
        switch (src)
        {
        case SIRC:
            srcFreq = SCG_get_sirc_freq();
            break;
        case FIRC:
            srcFreq = SCG_get_firc_freq();
            maximumBusFreq = RUN_MAXIMUM_BUS_FREQ_FIRC;
            break;
        case SPLL:
            srcFreq = SCG_get_spll_freq();
            maximumBusFreq = RUN_MAXIMUM_BUS_FREQ_SPLL;
            break;
        default:
            ret = eInvalidParameter;
            break;
        }

        if (0 != srcFreq)
        {
            /* Calculate best DIVCORE value */
            coreDivider = SCG_get_best_divider(coreFreq, srcFreq);
            if ((coreDivider >= 0) && (coreDivider < 16))
            {
                scg_reg_ptr = &(SCG->RCCR);
                mode = eRun;
            }
            else
            {
                scg_reg_ptr = NULL;
            }
            /* Calculate best DIVBUS value */
            busDivider = SCG_get_best_divider((busFreq <= maximumBusFreq) ?
                                                busFreq : maximumBusFreq, srcFreq);
            if ((busDivider < 0) || (busDivider >= 16))
            {
                ret = eInvalidParameter;
            }
            /* Calculate best DIVSLOW value */
            flashDivider = SCG_get_best_divider((flashFreq <= RUN_MAXIMUM_FLASH_FREQ) ?
                                                    flashFreq : RUN_MAXIMUM_FLASH_FREQ, srcFreq);
            if ((flashDivider < 0) || (flashDivider >= 16))
            {
                ret = eInvalidParameter;
            }
        }
        else
        {
            ret = eInvalidParameter;
        }
    }
    /* HSRun mode */
    else if (HSRUN_MAXIMUM_CORE_FREQ >= coreFreq)
    {
        /* clear source frequency */
        srcFreq &= 0;
        switch (src)
        {
        case SPLL:
            srcFreq = SCG_get_spll_freq();
            break;
        default:
            ret = eInvalidParameter;
            break;
        }

        if (0 != srcFreq)
        {
            /* Calculate best DIVCORE value */
            coreDivider = SCG_get_best_divider(coreFreq, srcFreq);
            if ((coreDivider >= 0) && (coreDivider < 16))
            {
                scg_reg_ptr = &(SCG->HCCR);
                mode = eHSRun;
            }
            else
            {
                scg_reg_ptr = NULL;
            }
            /* Calculate best DIVBUS value */
            busDivider = SCG_get_best_divider((busFreq <= HSRUN_MAXIMUM_BUS_FREQ) ?
                                                busFreq : HSRUN_MAXIMUM_BUS_FREQ, srcFreq);
            if ((busDivider < 0) || (busDivider >= 16))
            {
                ret = eInvalidParameter;
            }
            /* Calculate best DIVSLOW value */
            flashDivider = SCG_get_best_divider((flashFreq <= HSRUN_MAXIMUM_FLASH_FREQ) ?
                                                    flashFreq : HSRUN_MAXIMUM_FLASH_FREQ, srcFreq);
            if ((flashDivider < 0) || (flashDivider >= 16))
            {
                ret = eInvalidParameter;
            }
        }
        else
        {
            ret = eInvalidParameter;
        }
    }
    /* Invalid core frequency value */
    else
    {
        ret = eInvalidParameter;
    }

    if ((NULL != scg_reg_ptr) && (ret == eNoError))
    {
        /* Set DIVCORE value */
        *scg_reg_ptr =  SCG_CSR_SCS(src) |
                        SCG_CSR_DIVCORE(coreDivider) |
                        SCG_CSR_DIVBUS(busDivider) |
                        SCG_CSR_DIVSLOW(flashDivider);
        /* Switch to VLPR or HSRUN mode if needed */
        switch (mode)
        {
        case eVLPR:
            /* Allow very low power run mode */
            SMC->PMPROT |= SMC_PMPROT_AVLP_MASK;
            /* Check if current mode is RUN mode */
            if(SMC->PMSTAT == 0x01)
            {
                PMC->REGSC |= PMC_REGSC_BIASEN_MASK;
                /* Move to VLPR Mode*/
                SMC->PMCTRL = SMC_PMCTRL_RUNM(0b10);
                /* Wait for Transition*/
                while(SMC->PMSTAT != 0x04);
            }
            break;
        case eHSRun:
            /* Allow high speed run mode */
            SMC->PMPROT |= SMC_PMPROT_AHSRUN_MASK;
            /* Check if current mode is RUN mode */
            if(SMC->PMSTAT == 0x01)
            {
                /* Move to HSRUN Mode*/
                SMC->PMCTRL = SMC_PMCTRL_RUNM(0b11);
                /* Wait for Transition*/
                while(SMC->PMSTAT != 0x80);
            }
            break;
        case eRun:
            /* Check if current mode is HSRUN or VLPR mode */
            if((SMC->PMSTAT == 0x80) || (SMC->PMSTAT == 0x04))
            {
                /* Move to RUN Mode*/
                SMC->PMCTRL = SMC_PMCTRL_RUNM(0b00);
                /* Wait for Transition*/
                while(SMC->PMSTAT != 0x01);
            }
            break;
        default:
            break;
        }
    }
    return ret;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_best_divider
 *
 * @brief   Function to get the best divider value to meet the desired frequency
 *          from source frequency. If source frequency cannot use any divider to
 *          meet desired frequency, then, divider to get the closest frequency
 *          is used.
 *
 * @param   uint32_t desiredFreq: Desire frequency.
 *          uint32_t srcFreq: Value for source frequency.
 *
 * @return  int8_t: Best divider parameter, -1 if no divider matches.
 *****************************************************************************/
static int8_t SCG_get_best_divider (uint32_t desiredFreq, uint32_t srcFreq)
{
    int8_t divider, bestDivider = -1;
    uint8_t done = false;
    uint32_t difference, minError, calculatedFreq;

    /* Be sure that srcFreq is not zero */
    if (0 != srcFreq)
    {
        /* Iterate from all possible divider values */
        for (divider = 0, minError = desiredFreq; (divider < 16) && (false == done); divider++)
        {
            /* Calculate frequency for divider value */
            calculatedFreq = srcFreq / (divider + 1);

            /* Get difference between calculated frequency and desired frequency */
            difference = (desiredFreq > calculatedFreq) ? desiredFreq - calculatedFreq : calculatedFreq - desiredFreq;

            /* If difference is less than previous difference, then save this new minimum difference */
            if (difference < minError)
            {
                /* save current difference */
                minError = difference;
                /* save best divider */
                bestDivider = divider;
                /* No difference? */
                if (difference == 0)
                {
                    /* Finish the process*/
                    done = true;
                }
            }
        }
    }
    return bestDivider;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_firc_freq
 *
 * @brief   Get FIRC frequency value. If not enabled, return zero.
 *
 * @param   none.
 *
 * @return  uint32_t: FIRC frequency. Zero if FIRC is not enabled
 *****************************************************************************/
static uint32_t SCG_get_firc_freq(void)
{
    uint32_t freq = 0;
    if (((SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) >> SCG_FIRCCSR_FIRCVLD_SHIFT) == 1)
    {
        freq = FIRC_VALUE;
    }
    return freq;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_sirc_freq
 *
 * @brief   Get SIRC frequency value. If not enabled, return zero.
 *
 * @param   none.
 *
 * @return  uint32_t: SIRC frequency. Zero if SIRC is not enabled
 *****************************************************************************/
static uint32_t SCG_get_sirc_freq(void)
{
    uint32_t freq = 0;
    if (((SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD_MASK) >> SCG_SIRCCSR_SIRCVLD_SHIFT) == 1)
    {
        freq = SIRC_VALUE;
    }
    return freq;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_sosc_freq
 *
 * @brief   Get System Oscillator frequency value. If not enabled, return zero.
 *
 * @param   none.
 *
 * @return  uint32_t: SOSC frequency. Zero if SOSC is not enabled
 *****************************************************************************/
static uint32_t SCG_get_sosc_freq(void)
{
    uint32_t freq = 0;
    if (((SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) >> SCG_SOSCCSR_SOSCVLD_SHIFT) == 1)
    {
        freq = OSC_VALUE;
    }
    return freq;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_spll_freq
 *
 * @brief   Get System PLL frequency value. If not enabled, return zero.
 *
 * @param   none.
 *
 * @return  uint32_t: SPLL frequency. Zero if SPLL is not enabled
 *****************************************************************************/
static uint32_t SCG_get_spll_freq(void)
{
    uint32_t freq = 0;
    if (((SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK) >> SCG_SPLLCSR_SPLLVLD_SHIFT) == 1)
    {
        /* VCO = SOSC_CLK/(PREDIV + 1) X (MULT + 16) */
        /* SPLL_CLK = (VCO_CLK) / 2 */
        freq = ((SCG_get_sosc_freq() / (SPLL_PREDIV_VAL() + 1)) * (SPLL_MULT_VAL() + 16)) / 2;
    }
    return freq;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_core_freq
 *
 * @brief   Get Core frequency value.
 *
 * @param   none.
 *
 * @return  uint32_t: Core frequency
 *****************************************************************************/
static uint32_t SCG_get_core_freq(void)
{
    uint32_t freq = 0, div = (((SCG->CSR & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1);
    switch ((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT)
    {
    case SOSC:
        freq = SCG_get_sosc_freq() / div;
        break;
    case SIRC:
        freq = SCG_get_sirc_freq() / div;
        break;
    case FIRC:
        freq = SCG_get_firc_freq() / div;
        break;
    case SPLL:
        freq = SCG_get_spll_freq() / div;
        break;
    default:
        break;
    }
    return freq;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_bus_freq
 *
 * @brief   Get bus frequency value.
 *
 * @param   none.
 *
 * @return  uint32_t: bus frequency
 *****************************************************************************/
static uint32_t SCG_get_bus_freq(void)
{
    uint32_t div = (((SCG->CSR & SCG_CSR_DIVBUS_MASK) >> SCG_CSR_DIVBUS_SHIFT) + 1);
    return SCG_get_core_freq() / div;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_flash_freq
 *
 * @brief   Get flash frequency value.
 *
 * @param   none.
 *
 * @return  uint32_t: flash frequency
 *****************************************************************************/
static uint32_t SCG_get_flash_freq(void)
{
    uint32_t div = (((SCG->CSR & SCG_CSR_DIVSLOW_MASK) >> SCG_CSR_DIVSLOW_SHIFT) + 1);
    return SCG_get_core_freq() / div;
}

/**************************************************************************//*!
 *
 * @name    SCG_get_best_async_dividers
 *
 * @brief   Function to get the best dividers (DIV1/DIV2) value to meet the
 *          desired frequency from source (asynchronous) frequency. If source
 *          frequency cannot use any divider to meet desired frequency, then,
 *          divider to get the closest frequency is used.
 *
 * @param   uint32_t desiredFreq: Desire frequency.
 *          uint32_t srcFreq: Value for source frequency.
 *
 * @return  int8_t: Best divider parameter, -1 if no divider matches.
 *****************************************************************************/
static int8_t SCG_get_best_async_dividers (uint32_t desiredFreq, uint32_t srcFreq)
{
    int8_t divider, bestDivider = -1;
    uint8_t done = false;
    uint32_t difference, minError, calculatedFreq;

    /* Be sure that srcFreq is not zero */
    if (0 != srcFreq)
    {
        /* Iterate from all possible divider values */
        for (divider = 1, minError = desiredFreq; (divider < 8) && (false == done); divider++)
        {
            /* Calculate frequency for divider value */
            calculatedFreq = srcFreq / (1 << (divider - 1));

            /* Get difference between calculated frequency and desired frequency */
            difference = (desiredFreq > calculatedFreq) ? desiredFreq - calculatedFreq : calculatedFreq - desiredFreq;

            /* If difference is less than previous difference, then save this new minimum difference */
            if (difference < minError)
            {
                /* save current difference */
                minError = difference;
                /* save best divider */
                bestDivider = divider;
                /* No difference? */
                if (difference == 0)
                {
                    /* Finish the process*/
                    done = true;
                }
            }
        }
    }
    return bestDivider;
}

