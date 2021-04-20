/*
 * template_config.c
 *
 *  Created on: Feb 10, 2017
 *      Author: B50982
 */


#include "scg_config.h"

SCG_config_t configStruct = {
    .coreFreq       = 80000000,                /* Set core clock to 112 MHz */
    .busFreq        = 40000000,                 /* Set bus clock frequency to 56 MHz */
    .flashFreq      = 20000000,                 /* Set Flash frequency to 28 MHz */
};
