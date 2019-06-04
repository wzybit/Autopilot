
#ifndef FLASH_W25QXX_H
#define	FLASH_W25QXX_H

#include "stm32f4xx.h"

#define W25QXX_CS_LOW()      GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET)
#define W25QXX_CS_HIGH()     GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET)

#define JEDEC_MANUFACTURER_ST       0x20
#define JEDEC_MANUFACTURER_MACRONIX 0xC2
#define JEDEC_MANUFACTURER_WINBOND  0xEF

/* JEDEC Device ID: Memory type and Capacity */
#define JEDEC_W25Q16_BV_CL_CV   (0x4015) /* W25Q16BV W25Q16CL W25Q16CV  */
#define JEDEC_W25Q16_DW         (0x6015) /* W25Q16DW  */
#define JEDEC_W25Q32_BV         (0x4016) /* W25Q32BV */
#define JEDEC_W25Q32_DW         (0x6016) /* W25Q32DW */
#define JEDEC_W25Q64_BV_CV      (0x4017) /* W25Q64BV W25Q64CV */
#define JEDEC_W25Q64_DW         (0x4017) /* W25Q64DW */
#define JEDEC_W25Q128_BV        (0x4018) /* W25Q128BV */

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_SECTOR_ERASE           0x20

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

#define DUMMY_BYTE     0xFF


typedef struct
{
    uint8_t Manufacturer;             /* Manufacturer ID */
    uint8_t Memory;               /* Density Code */
    uint8_t Capacity;                /* Family Code */
    uint8_t rev;

} jedec_id_t;

/* Jedec Flash Information structure */
typedef struct
{
    uint8_t initialized;
    uint16_t sector_size;
    uint16_t sector_count;
    uint32_t capacity;
} flash_info_t;


#ifdef  __cplusplus  
extern "C" {  
#endif     
void Flash_w25qxx_Init(void);
void Flash_SectorErase(uint32_t address,uint8_t state);
void Flash_PageRead(uint32_t address,uint8_t* buffer,  uint32_t lenght);
void Flash_PageWrite(uint32_t address,uint8_t* buffer,  uint32_t lenght);
void Flash_SectorsRead(uint32_t address,uint8_t *buffer,uint16_t count);
void Flash_SectorsWrite(uint32_t address,uint8_t *buffer,uint16_t count);
flash_info_t *Flash_GetInfo(void);
#ifdef  __cplusplus  
}  
#endif  

#endif //FLASH_W25QXX_H
