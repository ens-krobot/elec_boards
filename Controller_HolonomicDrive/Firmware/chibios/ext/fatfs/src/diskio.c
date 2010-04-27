/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "ch.h"
#include "hal.h"

#include "diskio.h"

extern MMCDriver MMCD1;

/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */

#define MMC		0



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
    DSTATUS stat;

    switch (drv) {
	case MMC:
        stat = 0;
        /* It is initialized externally, just reads the status.*/
        if (mmcGetDriverState(&MMCD1) != MMC_READY)
            stat |= STA_NODISK;
        if (mmcIsWriteProtected(&MMCD1))
            stat |=  STA_PROTECT;
        return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat;

	switch (drv) {
	case MMC:
	    stat = 0;
        /* It is initialized externally, just reads the status.*/
        if (mmcGetDriverState(&MMCD1) != MMC_READY)
            stat |= STA_NODISK;
        if (mmcIsWriteProtected(&MMCD1))
            stat |=  STA_PROTECT;
        return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
	switch (drv) {
	case MMC:
        if (mmcGetDriverState(&MMCD1) != MMC_READY)
            return RES_NOTRDY;
        if (mmcStartSequentialRead(&MMCD1, sector))
            return RES_ERROR;
        while (count > 0) {
            if (mmcSequentialRead(&MMCD1, buff))
                return RES_ERROR;
            buff += MMC_SECTOR_SIZE;
            count--;
        }
        if (mmcStopSequentialRead(&MMCD1))
            return RES_ERROR;
		return RES_OK;
	}
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
	switch (drv) {
	case MMC:
        if (mmcGetDriverState(&MMCD1) != MMC_READY)
            return RES_NOTRDY;
        if (mmcIsWriteProtected(&MMCD1))
            return RES_WRPRT;
        if (mmcStartSequentialWrite(&MMCD1, sector))
            return RES_ERROR;
        while (count > 0) {
            if (mmcSequentialWrite(&MMCD1, buff))
                return RES_ERROR;
            buff += MMC_SECTOR_SIZE;
            count--;
        }
        if (mmcStopSequentialWrite(&MMCD1))
            return RES_ERROR;
		return RES_OK;
	}
	return RES_PARERR;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	switch (drv) {
	case MMC:
	    switch (ctrl) {
	    case CTRL_SYNC:
	        return RES_OK;
	    case GET_SECTOR_SIZE:
            *((WORD *)buff) = MMC_SECTOR_SIZE;
	        return RES_OK;
	    default:
	        return RES_PARERR;
	    }

		return RES_OK;
	}
	return RES_PARERR;
}

DWORD get_fattime(void) {

  return 0;
}
