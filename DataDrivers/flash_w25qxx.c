
#include "flash_w25qxx.h"
#include "stdio.h"

static flash_info_t flash_info;
static uint8_t Flash_SendByte ( uint8_t byte );
static void Flash_WaitForEnd ( void );
static uint8_t Flash_ReadID ( jedec_id_t *id );
static void Flash_WriteEnable ( void );

static uint8_t Flash_SendByte ( uint8_t byte )
{
    /* Loop while DR register in not emplty */
    while ( SPI_I2S_GetFlagStatus ( SPI1, SPI_I2S_FLAG_TXE ) == RESET );

    /* Send byte through the SPI1 peripheral */
    SPI_I2S_SendData ( SPI1, byte );

    /* Wait to receive a byte */
    while ( SPI_I2S_GetFlagStatus ( SPI1, SPI_I2S_FLAG_RXNE ) == RESET );

    /* Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData ( SPI1 );
}

void Flash_w25qxx_Init ( void )
{
    jedec_id_t flash_id;
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_SPI1, ENABLE );

    //SPI GPIO Configuration
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource5, GPIO_AF_SPI1 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource6, GPIO_AF_SPI1 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource7, GPIO_AF_SPI1 );

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

    //flash SPI CS
    GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode            = GPIO_Mode_OUT ;   //ÍÆÍìÊä³ö
    GPIO_InitStructure.GPIO_OType           = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd            = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed           = GPIO_Speed_2MHz;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

    //SPI configuration
    SPI_I2S_DeInit ( SPI1 );
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init ( SPI1, &SPI_InitStructure );
    SPI_Cmd ( SPI1, ENABLE );
    W25QXX_CS_HIGH();

    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send "0xff " instruction */
    Flash_SendByte ( DUMMY_BYTE );

    W25QXX_CS_HIGH();

    /* read flash id */
    Flash_ReadID ( &flash_id );

    if ( flash_id.Manufacturer == JEDEC_MANUFACTURER_WINBOND )
    {
        flash_info.sector_size = 4096;                         /* Page Erase (4096 Bytes) */
        if ( flash_id.Capacity == ( JEDEC_W25Q128_BV & 0xff ) )
        {
            flash_info.sector_count = 4096;                        /* 128Mbit / 8 / 4096 = 4096 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q64_DW & 0xff ) )
        {
            flash_info.sector_count = 2048;                       /* 64Mbit / 8 / 4096 = 2048 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q32_DW & 0xff ) )
        {
            flash_info.sector_count = 1024;                       /* 32Mbit / 8 / 4096 = 1024 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q16_DW & 0xff ) )
        {
            flash_info.sector_count = 512;                       /* 16Mbit / 8 / 4096 = 512 */
        }
        else
        {
            flash_info.sector_count = 0;
        }

        flash_info.capacity = flash_info.sector_size * flash_info.sector_count;
    }
    else
    {
        flash_info.initialized = 0;
        return ;
    }

    flash_info.initialized = 1;
}
static void Flash_WriteEnable ( void )
{
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send Write Enable instruction */
    Flash_SendByte ( JEDEC_WRITE_ENABLE );
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();
}

//Erases the specified FLASH sector.
void Flash_SectorErase ( uint32_t address, uint8_t state )
{
    Flash_WriteEnable();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send Sector Erase instruction */
    Flash_SendByte ( JEDEC_SECTOR_ERASE );
    /* Send SectorAddr high nibble address byte */
    Flash_SendByte ( ( address & 0xFF0000 ) >> 16 );
    /* Send SectorAddr medium nibble address byte */
    Flash_SendByte ( ( address & 0xFF00 ) >> 8 );
    /* Send SectorAddr low nibble address byte */
    Flash_SendByte ( address & 0xFF );
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    /* Wait the end of Flash writing */
    if ( state )
    {
        Flash_WaitForEnd();
    }

}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE
  *         cycle(Page WRITE sequence). The number of byte can't exceed
  *         the FLASH page size.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *                  written to the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH,
  *                       must be equal or less than "SPI_FLASH_PageSize" value.
  * @retval : None
  */
void Flash_PageWrite ( uint32_t address, uint8_t* buffer,  uint32_t lenght )
{
    Flash_WriteEnable();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send "Write to Memory " instruction */
    Flash_SendByte ( JEDEC_PAGE_WRITE );
    /* Send WriteAddr high nibble address byte to write to */
    Flash_SendByte ( ( address & 0xFF0000 ) >> 16 );
    /* Send WriteAddr medium nibble address byte to write to */
    Flash_SendByte ( ( address & 0xFF00 ) >> 8 );
    /* Send WriteAddr low nibble address byte to write to */
    Flash_SendByte ( address & 0xFF );

    /* while there is data to be written on the FLASH */
    while ( lenght-- )
    {
        /* Send the current byte */
        Flash_SendByte ( *buffer );
        /* Point on the next byte to be written */
        buffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    /* Wait the end of Flash writing */
    Flash_WaitForEnd();

}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param buffer : pointer to the buffer that receives the data read
  *                  from the FLASH.
  * @param address : FLASH's internal address to read from.
  * @param lenght : number of bytes to read from the FLASH.
  * @retval : None
  */
void Flash_PageRead ( uint32_t address, uint8_t* buffer,  uint32_t lenght )
{

    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();

    /* Send "Read from Memory " instruction */
    Flash_SendByte ( JEDEC_READ_DATA );

    /* Send ReadAddr high nibble address byte to read from */
    Flash_SendByte ( ( address & 0xFF0000 ) >> 16 );
    /* Send ReadAddr medium nibble address byte to read from */
    Flash_SendByte ( ( address & 0xFF00 ) >> 8 );
    /* Send ReadAddr low nibble address byte to read from */
    Flash_SendByte ( address & 0xFF );

    while ( lenght-- ) /* while there is data to be read */
    {
        /* Read a byte from the FLASH */
        *buffer = Flash_SendByte ( DUMMY_BYTE );
        /* Point to the next location where the byte read will be saved */
        buffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

}

//Reads FLASH identification.
uint8_t Flash_ReadID ( jedec_id_t *id )
{
    uint8_t *recv_buffer = ( uint8_t* ) id;

    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();

    /* Send "RDID " instruction */
    Flash_SendByte ( JEDEC_DEVICE_ID );

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte ( DUMMY_BYTE );

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte ( DUMMY_BYTE );

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte ( DUMMY_BYTE );

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    return id->Manufacturer;
}

static void Flash_WaitForEnd ( void )
{
    u8 FLASH_Status = 0;

    /* Loop as long as the memory is busy with a write cycle */
    do
    {
        /* Select the FLASH: Chip Select low */
        W25QXX_CS_LOW();
        /* Send "Read Status Register" instruction */
        Flash_SendByte ( JEDEC_READ_STATUS );
        /* Send a dummy byte to generate the clock needed by the FLASH
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = Flash_SendByte ( DUMMY_BYTE );
        /* Deselect the FLASH: Chip Select high */
        W25QXX_CS_HIGH();
    }
    while ( FLASH_Status & JEDEC_STATUS_BUSY );


}

void Flash_SectorsRead ( uint32_t address, uint8_t *buffer, uint16_t count )
{
    uint16_t i = 0;

    for ( i = 0; i < count; i++ )
    {
//		  page=flash_info.sector_size/256;
//		  while(page--)
//			{
        Flash_PageRead ( address, buffer, flash_info.sector_size );
        buffer += flash_info.sector_size;
        address += flash_info.sector_size;
//			}
    }
}
void Flash_SectorsWrite ( uint32_t address, uint8_t *buffer, uint16_t count )
{
    uint16_t i = 0, page = flash_info.sector_size / 256;
    Flash_WriteEnable();
    for ( i = 0; i < count; i++ )
    {
        Flash_SectorErase ( address, 1 );
        page = flash_info.sector_size / 256;
        while ( page-- )
        {
            Flash_PageWrite ( address, buffer, 256 );
            buffer += 256;
            address += 256;
        }
    }
}

flash_info_t *Flash_GetInfo ( void )
{
    return &flash_info;
}

