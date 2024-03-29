/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    mmc_spi.h
 * @brief   MMC over SPI driver header.
 *
 * @addtogroup MMC_SPI
 * @{
 */

#ifndef _MMC_SPI_H_
#define _MMC_SPI_H_

#if CH_HAL_USE_MMC_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define MMC_CMD0_RETRY          10
#define MMC_CMD1_RETRY          100
#define MMC_WAIT_DATA           10000

#define MMC_CMDGOIDLE           0
#define MMC_CMDINIT             1
#define MMC_CMDREADCSD          9
#define MMC_CMDSTOP             12
#define MMC_CMDSETBLOCKLEN      16
#define MMC_CMDREAD             17
#define MMC_CMDREADMULTIPLE     18
#define MMC_CMDWRITE            24
#define MMC_CMDWRITEMULTIPLE    25

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Block size for MMC transfers.
 */
#if !defined(MMC_SECTOR_SIZE) || defined(__DOXYGEN__)
#define MMC_SECTOR_SIZE         512
#endif

/**
 * @brief   Delays insertions.
 * @details If enabled this options inserts delays into the MMC waiting
 *          routines releasing some extra CPU time for the threads with
 *          lower priority, this may slow down the driver a bit however.
 *          This option is recommended also if the SPI driver does not
 *          use a DMA channel and heavily loads the CPU.
 */
#if !defined(MMC_NICE_WAITING) || defined(__DOXYGEN__)
#define MMC_NICE_WAITING        TRUE
#endif

/**
 * @brief   Number of positive insertion queries before generating the
 *          insertion event.
 */
#if !defined(MMC_POLLING_INTERVAL) || defined(__DOXYGEN__)
#define MMC_POLLING_INTERVAL    10
#endif

/**
 * @brief   Interval, in milliseconds, between insertion queries.
 */
#if !defined(MMC_POLLING_DELAY) || defined(__DOXYGEN__)
#define MMC_POLLING_DELAY       10
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !CH_HAL_USE_SPI || !CH_USE_EVENTS
#error "MMC_SPI driver requires CH_HAL_USE_SPI and CH_USE_EVENTS"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  MMC_UNINIT = 0,                           /**< @brief Not initialized.    */
  MMC_STOP = 1,                             /**< @brief Stopped.            */
  MMC_WAIT = 2,                             /**< @brief Waiting card.       */
  MMC_INSERTED = 3,                         /**< @brief Card inserted.      */
  MMC_READY = 4,                            /**< @brief Card ready.         */
  MMC_READING = 5,                          /**< @brief Reading.            */
  MMC_WRITING = 6                           /**< @brief Writing.            */
} mmcstate_t;

/**
 * @brief   Function used to query some hardware status bits.
 *
 * @return              The status.
 */
typedef bool_t (*mmcquery_t)(void);

/**
 * @brief   Driver configuration structure.
 */
typedef struct {

} MMCConfig;

/**
 * @brief   Structure representing a MMC driver.
 */
typedef struct {
  /**
   * @brief Driver state.
   */
  mmcstate_t            mmc_state;
  /**
   * @brief Current configuration data.
   */
  const MMCConfig       *mmc_config;
  /**
   * @brief SPI driver associated to this MMC driver.
   */
  SPIDriver             *mmc_spip;
  /**
   * @brief SPI low speed configuration used during initialization.
   */
  const SPIConfig       *mmc_lscfg;
  /**
   * @brief SPI high speed configuration used during transfers.
   */
  const SPIConfig       *mmc_hscfg;
  /**
   * @brief Write protect status query function.
   */
  mmcquery_t            mmc_is_protected;
  /**
   * @brief Insertion status query function.
   */
  mmcquery_t            mmc_is_inserted;
  /**
   * @brief Card insertion event source.
   */
  EventSource           mmc_inserted_event;
  /**
   * @brief Card removal event source.
   */
  EventSource           mmc_removed_event;
  /**
   * @brief MMC insertion polling timer.
   */
  VirtualTimer          mmc_vt;
  /**
   * @brief Insertion counter.
   */
  uint_fast8_t          mmc_cnt;
} MMCDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the driver state.
 */
#define mmcGetDriverState(mmcp) ((mmcp)->mmc_state)

/**
 * @brief   Returns the write protect status.
 */
#define mmcIsWriteProtected(mmcp) ((mmcp)->mmc_is_protected())

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void mmcInit(void);
  void mmcObjectInit(MMCDriver *mmcp, SPIDriver *spip,
                     const SPIConfig *lscfg, const SPIConfig *hscfg,
                     mmcquery_t is_protected, mmcquery_t is_inserted);
  void mmcStart(MMCDriver *mmcp, const MMCConfig *config);
  void mmcStop(MMCDriver *mmcp);
  bool_t mmcConnect(MMCDriver *mmcp);
  bool_t mmcDisconnect(MMCDriver *mmcp);
  bool_t mmcStartSequentialRead(MMCDriver *mmcp, uint32_t startblk);
  bool_t mmcSequentialRead(MMCDriver *mmcp, uint8_t *buffer);
  bool_t mmcStopSequentialRead(MMCDriver *mmcp);
  bool_t mmcStartSequentialWrite(MMCDriver *mmcp, uint32_t startblk);
  bool_t mmcSequentialWrite(MMCDriver *mmcp, const uint8_t *buffer);
  bool_t mmcStopSequentialWrite(MMCDriver *mmcp);
#ifdef __cplusplus
}
#endif

#endif /* CH_HAL_USE_MMC_SPI */

#endif /* _MMC_SPI_H_ */

/** @} */
