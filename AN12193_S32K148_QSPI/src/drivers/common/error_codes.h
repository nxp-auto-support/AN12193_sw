/*
 * error_codes.h
 *
 *  Rev:
 *  Author:
 *  Date:
 */

#ifndef ERROR_CODES_H_
#define ERROR_CODES_H_

typedef enum {
    eNoError = 0,
    eInvalidParameter,
    eNullPointer,
    eModuleIsBusy
}eErrorCodes;

#ifndef NULL
#define NULL    (void *)0
#endif

#ifndef true
#define true    (uint8_t)1
#endif

#ifndef false
#define false    (uint8_t)0
#endif

#endif /* ERROR_CODES_H_ */
