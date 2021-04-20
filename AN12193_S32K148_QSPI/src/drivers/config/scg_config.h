/*
 * template_config.h
 *
 *  Rev:
 *  Author:
 *  Date:
 */

#ifndef SCG_CONFIG_H_
#define SCG_CONFIG_H_

#include "system.h"

#define MAXIMUM_AVAILABLE_FREQ  (0xFFFFFFFF)

/* SCG configuration structure: it contains configuration
 * settings for different clock sources available on SCG module.
 * This structure contains frequencies for core, bus and flash
 * besides the asynchronous clock sources */
typedef struct {
    uint32_t    coreFreq;       /* Desired frequency for core */
    uint32_t    busFreq;        /* Bus clock frequency */
    uint32_t    flashFreq;      /* Flash frequency */
}SCG_config_t;

#endif /* SCG_CONFIG_H_ */
