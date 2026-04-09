/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs - SD Card implementation          */
/*-----------------------------------------------------------------------*/

#include "ff.h"
#include "diskio.h"

// SD卡驱动头文件
#include "sdcard.h"

/* Definitions of physical drive number for each drive */
#define DEV_SD      0   /* SD卡映射到物理驱动0 */

/* SD卡状态标志 */
static DSTATUS sd_stat = STA_NOINIT;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != DEV_SD)
        return STA_NOINIT;

    // 检查SD卡是否已初始化
    if (sd_stat == STA_NOINIT)
        return sd_stat;

    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                   */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv)
{
    sd_error_status_type ret;

    if (pdrv != DEV_SD)
        return STA_NOINIT;

    // 初始化SD卡
    ret = sd_init();
    if (ret != SD_OK)
    {
        sd_stat = STA_NOINIT;
        return sd_stat;
    }

    // 设置DMA模式
    sd_device_mode_set(SD_TRANSFER_DMA_MODE);

    sd_stat = 0;  // 清除NOINIT标志
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read(
    BYTE pdrv,       /* Physical drive number */
    BYTE *buff,      /* Data buffer to store read data */
    LBA_t sector,   /* Start sector in LBA */
    UINT count       /* Number of sectors to read */
)
{
    sd_error_status_type ret;

    if (pdrv != DEV_SD || sd_stat & STA_NOINIT)
        return RES_NOTRDY;

    if (count == 1)
    {
        ret = sd_block_read(buff, (long long)sector * 512, 512);
    }
    else
    {
        ret = sd_mult_blocks_read(buff, (long long)sector * 512, 512, count);
    }

    if (ret == SD_OK)
        return RES_OK;
    else
        return RES_ERROR;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if FF_FS_READONLY == 0
DRESULT disk_write(
    BYTE pdrv,           /* Physical drive number */
    const BYTE *buff,    /* Data to be written */
    LBA_t sector,       /* Start sector in LBA */
    UINT count           /* Number of sectors to write */
)
{
    sd_error_status_type ret;

    if (pdrv != DEV_SD || sd_stat & STA_NOINIT)
        return RES_NOTRDY;

    if (count == 1)
    {
        ret = sd_block_write(buff, (long long)sector * 512, 512);
    }
    else
    {
        ret = sd_mult_blocks_write(buff, (long long)sector * 512, 512, count);
    }

    if (ret == SD_OK)
        return RES_OK;
    else
        return RES_ERROR;
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(
    BYTE pdrv,       /* Physical drive number */
    BYTE cmd,        /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
    sd_card_info_struct_type card_info;

    if (pdrv != DEV_SD || sd_stat & STA_NOINIT)
        return RES_NOTRDY;

    switch (cmd)
    {
    case CTRL_SYNC:
        return RES_OK;

    case GET_SECTOR_COUNT:
        sd_card_info_get(&card_info);
        *(LBA_t *)buff = card_info.card_capacity / 512;
        return RES_OK;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 512;
        return RES_OK;

    default:
        return RES_PARERR;
    }
}
