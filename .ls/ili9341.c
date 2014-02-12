/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    ili9341.c
 * @brief   ILI9341 TFT LCD diaplay controller driver.
 * @note    Does not support multiple calling threads natively.
 */

#include "ch.h"
#include "hal.h"
#include "ili9341.h"

/**
 * @addtogroup ili9341
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if !ILI9341_USE_CHECKS && !defined(__DOXYGEN__)
/* Disable checks as needed.*/

#ifdef chDbgCheck
#undef chDbgCheck
#endif
#define chDbgCheck(c, func) {                                               \
  (void)(c), (void)__QUOTE_THIS(func)"()";                                  \
}

#ifdef chDbgAssert
#undef chDbgAssert
#endif
#define chDbgAssert(c, m, r) {                                              \
  (void)(c);                                                                \
}

#ifdef chDbgCheckClassS
#undef chDbgCheckClassS
#endif
#define chDbgCheckClassS() {}

#ifdef chDbgCheckClassS
#undef chDbgCheckClassS
#endif
#define chDbgCheckClassI() {}

#endif /* ILI9341_USE_CHECKS */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief ILI9341D1 driver identifier.*/
ILI9341Driver ILI9341D1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the standard part of a @p ILI9341Driver structure.
 *
 * @param[out] driverp  pointer to the @p ILI9341Driver object
 *
 * @init
 */
void ili9341ObjectInit(ILI9341Driver *driverp) {

  chDbgCheck(driverp != NULL, "ili9341ObjectInit");

  driverp->state = ILI9341_STOP;
  driverp->config = NULL;
#if ILI9341_USE_MUTUAL_EXCLUSION
#if CH_USE_MUTEXES
  chMtxInit(&driverp->lock);
#else
  chSemInit(&driverp->lock, 1);
#endif
#endif /* ILI9341_USE_MUTUAL_EXCLUSION */
}

/**
 * @brief   Configures and activates the ILI9341 peripheral.
 * @pre     ILI9341 is stopped.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 * @param[in] configp   pointer to the @p ILI9341Config object
 *
 * @api
 */
void ili9341Start(ILI9341Driver *driverp, const ILI9341Config *configp) {

  chSysLock();
  chDbgCheck(driverp != NULL, "ili9341Start");
  chDbgCheck(configp != NULL, "ili9341Start");
  chDbgCheck(configp->spi != NULL, "ili9341Start");
  chDbgAssert(driverp->state == ILI9341_STOP,
              "ili9341Start(), #1", "invalid state");

  spiSelectI(configp->spi);
  spiUnselectI(configp->spi);
  driverp->config = configp;
  driverp->state = ILI9341_READY;
  chSysUnlock();
}

/**
 * @brief   Deactivates the ILI9341 peripheral.
 * @pre     ILI9341 is ready.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @api
 */
void ili9341Stop(ILI9341Driver *driverp) {

  chSysLock();
  chDbgCheck(driverp != NULL, "ili9341Stop");
  chDbgAssert(driverp->state == ILI9341_READY,
              "ili9341Start(), #1", "invalid state");

  driverp->state = ILI9341_STOP;
  chSysUnlock();
}

#if ILI9341_USE_MUTUAL_EXCLUSION

/**
 * @brief   Gains exclusive access to the ILI9341 module.
 * @details This function tries to gain ownership to the ILI9341 module, if the
 *          module is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option
 *          @p ILI9341_USE_MUTUAL_EXCLUSION must be enabled.
 * @pre     ILI9341 is ready.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @sclass
 */
void ili9341AcquireBusS(ILI9341Driver *driverp) {

  chDbgCheckClassS();
  chDbgCheck(driverp == &ILI9341D1, "ili9341AcquireBusS");
  chDbgAssert(driverp->state == ILI9341_READY,
              "ili9341AcquireBusS(), #1", "not ready");

#if CH_USE_MUTEXES
  chMtxLockS(&driverp->lock);
#else
  chSemWaitS(&driverp->lock);
#endif
}

/**
 * @brief   Gains exclusive access to the ILI9341 module.
 * @details This function tries to gain ownership to the ILI9341 module, if the
 *          module is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option
 *          @p ILI9341_USE_MUTUAL_EXCLUSION must be enabled.
 * @pre     ILI9341 is ready.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @api
 */
void ili9341AcquireBus(ILI9341Driver *driverp) {

  chSysLock();
  ili9341AcquireBusS(driverp);
  chSysUnlock();
}

/**
 * @brief   Releases exclusive access to the ILI9341 module.
 * @pre     In order to use this function the option
 *          @p ILI9341_USE_MUTUAL_EXCLUSION must be enabled.
 * @pre     ILI9341 is ready.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @sclass
 */
void ili9341ReleaseBusS(ILI9341Driver *driverp) {

  chDbgCheckClassS();
  chDbgCheck(driverp == &ILI9341D1, "ili9341ReleaseBusS");
  chDbgAssert(driverp->state == ILI9341_READY,
              "ili9341ReleaseBusS(), #1", "not ready");

#if CH_USE_MUTEXES
  chMtxUnlockS();
#else
  chSemSignalI(&driverp->lock);
#endif
}

/**
 * @brief   Releases exclusive access to the ILI9341 module.
 * @pre     In order to use this function the option
 *          @p ILI9341_USE_MUTUAL_EXCLUSION must be enabled.
 * @pre     ILI9341 is ready.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @api
 */
void ili9341ReleaseBus(ILI9341Driver *driverp) {

  chSysLock();
  ili9341ReleaseBusS(driverp);
  chSysUnlock();
}

#endif /* ILI9341_USE_MUTUAL_EXCLUSION */

#if ILI9341_IM == ILI9341_IM_4LSI_1 /* 4-wire, half-duplex */

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 * @pre     ILI9341 is ready.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @iclass
 */
void ili9341SelectI(ILI9341Driver *driverp) {

  chDbgCheckClassI();
  chDbgCheck(driverp != NULL, "ili9341SelectI");
  chDbgAssert(driverp->state == ILI9341_READY,
              "ili9341SelectI(), #1", "invalid state");

  driverp->state = ILI9341_ACTIVE;
  spiSelectI(driverp->config->spi);
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 * @pre     ILI9341 is ready.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @api
 */
void ili9341Select(ILI9341Driver *driverp) {

  chSysLock();
  ili9341SelectI(driverp);
  chSysUnlock();
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 * @pre     ILI9341 is active.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @iclass
 */
void ili9341UnselectI(ILI9341Driver *driverp) {

  chDbgCheckClassI();
  chDbgCheck(driverp != NULL, "ili9341Unselect");
  chDbgAssert(driverp->state == ILI9341_ACTIVE,
              "ili9341UnselectI(), #1", "invalid state");

  spiUnselectI(driverp->config->spi);
  driverp->state = ILI9341_READY;
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 * @pre     ILI9341 is active.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @iclass
 */
void ili9341Unselect(ILI9341Driver *driverp) {

  chSysLock();
  ili9341UnselectI(driverp);
  chSysUnlock();
}

/**
 * @brief   Write command byte.
 * @details Sends a command byte via SPI.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 * @param[in] cmd       command byte
 *
 * @api
 */
void ili9341WriteCommand(ILI9341Driver *driverp, uint8_t cmd) {

  chDbgCheck(driverp != NULL, "ili9341WriteCommand");
  chDbgAssert(driverp->state == ILI9341_ACTIVE,
              "ili9341WriteCommand(), #1", "invalid state");

  driverp->value = cmd;
  palClearPad(driverp->config->dcx_port, driverp->config->dcx_pad); /* !Cmd */
  spiSend(driverp->config->spi, 1, &driverp->value);
}

/**
 * @brief   Write data byte.
 * @details Sends a data byte via SPI.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 * @param[in] value     data byte
 *
 * @api
 */
void ili9341WriteByte(ILI9341Driver *driverp, uint8_t value) {

  chDbgCheck(driverp != NULL, "ili9341WriteByte");
  chDbgAssert(driverp->state == ILI9341_ACTIVE,
              "ili9341WriteByte(), #1", "invalid state");

  driverp->value = value;
  palSetPad(driverp->config->dcx_port, driverp->config->dcx_pad); /* Data */
  spiSend(driverp->config->spi, 1, &driverp->value);
}

/**
 * @brief   Read data byte.
 * @details Receives a data byte via SPI.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 *
 * @return              data byte
 *
 * @api
 */
uint8_t ili9341ReadByte(ILI9341Driver *driverp) {

  chDbgAssert(FALSE, "ili9341ReadByte()", "should not be used");

  chDbgCheck(driverp != NULL, "ili9341ReadByte");
  chDbgAssert(driverp->state == ILI9341_ACTIVE,
              "ili9341ReadByte(), #1", "invalid state");

  palSetPad(driverp->config->dcx_port, driverp->config->dcx_pad); /* Data */
  spiReceive(driverp->config->spi, 1, &driverp->value);
  return driverp->value;
}

/**
 * @brief   Write data chunk.
 * @details Sends a data chunk via SPI.
 * @pre     The chunk must be accessed by DMA.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 * @param[in] chunk     chunk bytes
 * @param[in] length    chunk length
 *
 * @api
 */
void ili9341WriteChunk(ILI9341Driver *driverp, const uint8_t chunk[],
                       size_t length) {

  chDbgCheck(driverp != NULL, "ili9341WriteChunk");
  chDbgCheck(chunk != NULL, "ili9341WriteChunk");
  chDbgAssert(driverp->state == ILI9341_ACTIVE,
              "ili9341WriteChunk(), #1", "invalid state");

  if (length == 0)
    return;

  palSetPad(driverp->config->dcx_port, driverp->config->dcx_pad); /* Data */
  spiSend(driverp->config->spi, length, chunk);
}

/**
 * @brief   Read data chunk.
 * @details Receives a data chunk via SPI.
 * @pre     The chunk must be accessed by DMA.
 *
 * @param[in] driverp   pointer to the @p ILI9341Driver object
 * @param[out] chunk    chunk bytes
 * @param[in] length    chunk length
 *
 * @api
 */
void ili9341ReadChunk(ILI9341Driver *driverp, uint8_t chunk[],
                      size_t length) {

  chDbgCheck(driverp != NULL, "ili9341ReadChunk");
  chDbgCheck(chunk != NULL, "ili9341ReadChunk");
  chDbgAssert(driverp->state == ILI9341_ACTIVE,
              "ili9341ReadChunk(), #1", "invalid state");

  if (length == 0)
    return;

  palSetPad(driverp->config->dcx_port, driverp->config->dcx_pad); /* Data */
  spiReceive(driverp->config->spi, length, chunk);
}

#else /* ILI9341_IM == * */
#error "Only the ILI9341_IM_4LSI_1 interface mode is currently supported"
#endif /* ILI9341_IM == * */

/** @} */
