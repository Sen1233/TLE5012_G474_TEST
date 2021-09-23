/*
 * tle5012b_drive.c
 *
 *  Created on: 2021年6月19日
 *      Author: EE-Sen-Matebook
 */

#include "main.h"
#include "tle5012b_drive.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"

#include <string.h> // 内存拷贝

/* 包含SPI2外设组件 */
extern SPI_HandleTypeDef hspi2;

TLE5012_Handle_t  Abs_TLE5012_M1 =
{
    .SPIx                       =   SPI2,                                           // 使用SPI2来读写磁编芯片
    .SPI_CS_Port                =   M1_MAIN_ENCODER_CS1_GPIO_Port,                      // 磁编的使能管脚
    .SPI_CS_Pin                 =   M1_MAIN_ENCODER_CS1_Pin,                            // 磁编的使能管脚
    .dir                        =   CCW,                                            // 磁编的方向
    .error_code                 =   0U,                                             // 读写磁编时检测到的错误码
    .zero                       =   0U,                                             // 绝对零角度值
    .set_zero_ok                =   false,                                          // 绝对零位设置标志位
    .SSC_send_and_recv          =   &SPI2_B11_send_and_recv,                        // 关联函数调用
};

/**
  * @brief  SPI2的二次初始化
  *         第一次初始化是自动生成的, 工作在 5.3125MBits/s , 此初始化用来完成磁编的初始设置
  *         第二次初始化需要调用以下函数, 使磁编工作在高速模式下
  * @param  p_send_data 发送的数据指针
  * @param  send_len    发送数据的长度
  * @param  p_recv_data 接收数据的指针
  * @param  recv_len    需要接收数据的长度
  * @retval 包装过的HAL库API错误状态
  */
void New_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_1LINE;
    hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  调用HAL阻塞模式API, 交换数据
  *         更具实际需求改动, 此部分应该挪到SPI相关的文件内
  *         警告, 多线程同时调用会出现严重的问题, 因为执行对象就一个
  * @param  p_send_data 发送的数据指针
  * @param  send_len    发送数据的长度
  * @param  p_recv_data 接收数据的指针
  * @param  recv_len    需要接收数据的长度
  * @retval 包装过的HAL库API错误状态
  */
uint16_t SPI2_B11_send_and_recv( uint16_t * p_send_data, uint16_t * p_recv_data, uint16_t send_len, uint16_t recv_len )
{
    // 使能外设
    HAL_GPIO_WritePin( M1_MAIN_ENCODER_CS1_GPIO_Port, M1_MAIN_ENCODER_CS1_Pin, GPIO_PIN_RESET);

    // 主机使用SCK, MOSI, CS, 三个管脚
    // 注意电阻串联方式
    if ( HAL_OK != HAL_SPI_Transmit( &hspi2, (uint8_t*)p_send_data, send_len, 2U ))
        return TLE5012_ERROR_API;

    if ( HAL_OK != HAL_SPI_Receive( &hspi2, (uint8_t*)p_recv_data, recv_len, 2U ))
        return TLE5012_ERROR_API;

    // 关闭外设
    HAL_GPIO_WritePin( M1_MAIN_ENCODER_CS1_GPIO_Port, M1_MAIN_ENCODER_CS1_Pin, GPIO_PIN_SET);

    return TLE5012_ERROR_OK;
}

/*************************************************************/
/*************** 基于 LL 库实现快速的读写, 起始 ***************/
/************************************************************/

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  调用LL库函数实现SPI数据发送
  * @param  tx_data 需要发送出去的数据, 一个有效数据
  * @retval 返回是否发生异常
  */
static __inline bool SSC_LL_send( TLE5012_Handle_t * p_tle_dev, uint16_t tx_data )
{
    uint16_t retry = 0U;

    // 调整SPI的方向, 发送
    LL_SPI_Disable( p_tle_dev->SPIx );                                      // 关闭SPI外设
    LL_SPI_SetTransferDirection( p_tle_dev->SPIx, LL_SPI_HALF_DUPLEX_TX );  // 把GPIO (MOSI) 配置成推挽--输出模式 (50MHz)
    LL_SPI_Enable( p_tle_dev->SPIx );                                       // 开启SPI外设
    
    // 发送需要发送的数据
   	LL_SPI_TransmitData16( p_tle_dev->SPIx, tx_data);                       // 发送数据

    // 判断数据是否发送完成, 形成闭环
    while ( LL_SPI_IsActiveFlag_TXE( p_tle_dev->SPIx ) == 0U )              // 首先等待发送缓冲区为空
        if ( ++retry > 20U ) return false;                                  // 一直非空延迟一段时间后返回
    
    while ( LL_SPI_IsActiveFlag_BSY( p_tle_dev->SPIx ) != 0U )              // 其次等待SPI外设空闲
        if ( ++retry > 20U ) return false;                                  // 一直忙碌延时一段时间后返回

    // 完成发送任务
    return true;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  调用LL库函数实现SPI数据接收
  * @param  rx_data 接收到的数据返回指针
  * @param  size 需要连续接收数据的个数
  * @retval 返回是否发生异常
  */
static __inline bool SSC_LL_recv( TLE5012_Handle_t * p_tle_dev, uint16_t * rx_data, uint8_t size )
{
    uint16_t retry, index;

    // 调整SPI的方向, 接收
    LL_SPI_Disable( p_tle_dev->SPIx );                                      // 关闭SPI外设
    LL_SPI_SetTransferDirection( p_tle_dev->SPIx, LL_SPI_HALF_DUPLEX_RX );  // 把GPIO (MOSI) 配置成开漏--输入模式
    LL_SPI_ReceiveData16( p_tle_dev->SPIx );                                // 清除接收标志
    LL_SPI_Enable( p_tle_dev->SPIx );                                       // 开启SPI外设

    /* 当需要接收的数据大于1U时, 执行内部循环 */
    for ( index = 0U; index < size - 1U; index++ )
    {
        // 判断数据是否收到
        retry = 0U;
        while ( LL_SPI_IsActiveFlag_RXNE( p_tle_dev->SPIx ) == 0U )         // 接收缓冲区为空
            if ( ++retry > 20U ) return false;                              // 延迟一段时间后返回

        rx_data[index] = LL_SPI_ReceiveData16( p_tle_dev->SPIx );           // 读取有效数据并清除标志
    }

    // 当本次接收完成后不在继续接收, 提前关闭
    LL_SPI_Disable( p_tle_dev->SPIx );                                      // 关闭SPI外设

    // 判断数据是否收到
    retry = 0U;
    while ( LL_SPI_IsActiveFlag_RXNE( p_tle_dev->SPIx ) == 0U )             // 接收缓冲区为空
        if ( ++retry > 20U ) return false;                                  // 延迟一段时间后返回

    // // 当本次接收完成后不在继续接收, 提前关闭
    // LL_SPI_Disable( p_tle_dev->SPIx );                                      // 关闭SPI外设
    
    rx_data[index] = LL_SPI_ReceiveData16( p_tle_dev->SPIx );               // 读取有效数据并清除标志

    // 完成接收任务
    return true;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  获取寄存器值
  * @param  reg_value 控制字
  * @param  value     读取到的原始数据存放指针
  * @param  size      需要读取的数据个数
  * @retval 无
  */
void TLE5012_read_value( TLE5012_Handle_t * p_tle_dev, uint16_t reg_value, uint16_t * value, uint8_t size)
{
    /* 临时关闭全局中断 */
    __disable_irq();

    /* 使能磁编 */
    LL_GPIO_ResetOutputPin( p_tle_dev->SPI_CS_Port, p_tle_dev->SPI_CS_Pin);

    /* 执行发送任务 */
    SSC_LL_send( p_tle_dev, reg_value | size );

    /* 执行接收任务 */
    SSC_LL_recv( p_tle_dev, value, size );

    /* 关闭磁编 */
    LL_GPIO_SetOutputPin( p_tle_dev->SPI_CS_Port, p_tle_dev->SPI_CS_Pin);

    /* 从新开启全局中断 */
    __enable_irq();
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  获取角度, 得到 0~359 度
  * @param  angle_valuel 读取到的原始数据存放指针
  * @retval 无
  */
void TLE5012_read_angle( TLE5012_Handle_t * p_tle_dev, uint16_t * angle_valuel )
{
    uint16_t tmp[1U];
    TLE5012_read_value( p_tle_dev, SSC_READ_AVAL, tmp, 1U );
    *angle_valuel = *tmp & 0x7FFFU;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  获取角速度
  * @param  angle_speed 读取到的原始数据存放指针
  * @retval 无
  */
void TLE5012_read_speed( TLE5012_Handle_t * p_tle_dev, uint16_t * angle_speed )
{
    uint16_t tmp[1U];
    TLE5012_read_value( p_tle_dev, SSC_READ_ASPD, tmp, 1U );
    *angle_speed = *tmp & 0x7FFFU;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  获取角度和角速度双值
  * @param  angle_valuel 读取到的原始数据存放指针
  * @param  angle_speed  读取到的原始数据存放指针
  * @retval 无
  */
void TLE5012_read_angle_and_speed( TLE5012_Handle_t * p_tle_dev, uint16_t * angle_valuel, uint16_t * angle_speed )
{
    uint16_t tmp[2U];
    TLE5012_read_value( p_tle_dev, SSC_READ_AVAL, tmp, 2U );
    *angle_speed = tmp[1U] & 0x7FFFU;
    *angle_valuel = tmp[0U] & 0x7FFFU;
}

/*************************************************************/
/*************** 基于 LL 库实现快速的读写, 结尾 ***************/
/************************************************************/


/************************************************************/
/*************** 基于 HAL 库实现配置校验, 起始 ***************/
/***********************************************************/

/* 用于CRC计算的表值, 在 .c文件中实现 */
// 用于CRC计算的表值
// Look-up table (LUT) for the TLE5012B with generator polynomial 100011101 (0x11D).
// As this table will be checked byte by byte, each byte has 256 possible values (2^8)
// for its CRC calculation with the given generator polynomial.
// static const
static const uint8_t TableCRC[256] =
{
    // The “crc” of the position [1] (result from operation [crc ^*(message+Byteidx)])
    // is 0x00 -> 0x00 XOR 0x11D = 0x00 (1 byte).
    0x00,
    // The “crc” of the position [2] is 0x1D -> 0x01 XOR 0x11D = 0x1D (1 byte).
    0x1D,
    // The “crc” of the position [3] is 0x3A -> 0x02 XOR 0x11D = 0x3A (1 byte).
    0x3A,
    // For all the rest of the cases.
    0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB, 0xCD,
    0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B,
    0x76, 0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B,
    0x06, 0x21, 0x3C, 0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98,
    0x85, 0xD6, 0xCB, 0xEC, 0xF1, 0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB,
    0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90,
    0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65, 0x94, 0x89, 0xAE, 0xB3, 0xE0,
    0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F, 0x59, 0x44, 0x63,
    0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 0x26,
    0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80,
    0x9D, 0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77,
    0x6A, 0x4D, 0x50, 0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73,
    0x6E, 0x3D, 0x20, 0x07, 0x1A, 0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84,
    0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7, 0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B,
    0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E, 0xF8, 0xE5, 0xC2, 0xDF, 0x8C,
    0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 0xB2, 0xAF, 0x88,
    0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09, 0x7F,
    0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFe,
    // The “crc” of the position [255] is 0xD9 -> 0xFE XOR 0x11D = 0xD9 (1 byte).
    0xD9,
    // The “crc” of the position [256] is 0xC4 -> 0xFF XOR 0x11D = 0xC4 (1 byte).
    0xC4
};

/* CRC 算法一, 通过查表直接获取, 在 .c文件中实现 */
// “message” is the data transfer for which a CRC has to be calculated.
// A typical “message” consists of 2 bytes for the command word plus 2 bytes for the data word plus 2 bytes for the safety word.
// “Bytelength” is the number of bytes in the “message”. A typical “message” has 6 bytes.
// *Table CRC is the pointer to the look-up table (LUT)
static __inline uint8_t CRC8_TABLE( TLE5012_DATA_t * p_buffer, const uint8_t * TableCRC )
{
    uint16_t *tx_data = p_buffer->send_data;
    uint16_t *rx_data = p_buffer->recv_data;
    uint16_t txlength = p_buffer->send_len;
    uint16_t rxlength = p_buffer->recv_len;

    // “crc” defined as the 8-bits that will be generated through the message till the final crc is generated.
    // In the example above this are the blue lines out of the XOR operation.
    // Initially the CRC remainder has to be set with the original seed (0xFF for the TLE5012B).
    uint8_t crc = 0xFFU;
    // “Byteidx” is a counter to compare the bytes used for the CRC calculation and “Bytelength”.
    uint8_t Byteidx;
    
    uint8_t two_byte_idx, tmp;
    uint8_t message[32U] = {0U};
    uint8_t Bytelength = 0U;

    // 整理数据, 发送部分
    for ( two_byte_idx = 0U; two_byte_idx < txlength; two_byte_idx++ )
    {
        message[Bytelength++] = tx_data[two_byte_idx] >> 8U;
        message[Bytelength++] = tx_data[two_byte_idx] & 0xFFU;
    }
    // 整理数据, 接收部分
    for ( two_byte_idx = 0U; two_byte_idx < rxlength; two_byte_idx++ )
    {
        message[Bytelength++] = rx_data[two_byte_idx] >> 8U;
        message[Bytelength++] = rx_data[two_byte_idx] & 0xFFU;
    }

    // 将接收到的 CRC 数据暂存起来
    // 整个安全字不参与CRC的计算
    tmp = message[--Bytelength];
    --Bytelength;

    // For all the bytes of the message.
    for ( Byteidx = 0; Byteidx < Bytelength; Byteidx++ )
    {
        // “crc” is the value in the look-up table TableCRC[x] at the position “x”.
        // The position “x” is determined as the XOR operation between the previous “crc” and the next byte of the “message”.
        // “^” is the XOR operator.
        crc = TableCRC[crc ^ *( message+Byteidx )];
    }

    // Return the inverted “crc” remainder(“~” is the invertion operator).
    // An alternative to the “~” operator would be a XOR operation between “crc” and a 0xFF polynomial.
    return (( ~crc ) & 0xFFU ) != tmp;
}

/* CRC 算法二, 通过两个循环硬算, 在 .c文件中实现 */
// “message” is the data transfer for which a CRC has to be calculated.
// A typical “message” consists of 2 bytes for the command word plus 2 bytes for the data word plus 2 bytes for the safety word.
// “Bytelength” is the number of bytes in the “message”. A typical “message” has 6 bytes.
static __inline uint8_t CRC8( TLE5012_DATA_t * p_buffer )
{
    uint16_t *tx_data = p_buffer->send_data;
    uint16_t *rx_data = p_buffer->recv_data;
    uint16_t txlength = p_buffer->send_len;
    uint16_t rxlength = p_buffer->recv_len;

    // “crc” defined as the 8-bits that will be generated through the message till the final crc is generated.
    // In the example above this are the blue lines out of the XOR operation.
    // Initially the CRC remainder has to be set with the original seed (0xFF for the TLE5012B).
    uint8_t crc = 0xFFU;
    // “Byteidx” is a counter to compare the bytes used for the CRC calculation
    uint8_t Byteidx, Bitidx;

    uint8_t two_byte_idx, tmp;
    uint8_t message[32U] = {0U};
    uint8_t Bytelength = 0U;

    // 整理数据, 发送部分
    for ( two_byte_idx = 0U; two_byte_idx < txlength; two_byte_idx++ )
    {
        message[Bytelength++] = tx_data[two_byte_idx] >> 8U;
        message[Bytelength++] = tx_data[two_byte_idx] & 0xFFU;
    }
    // 整理数据, 接收部分
    for ( two_byte_idx = 0U; two_byte_idx < rxlength; two_byte_idx++ )
    {
        message[Bytelength++] = rx_data[two_byte_idx] >> 8U;
        message[Bytelength++] = rx_data[two_byte_idx] & 0xFFU;
    }

    // 将接收到的 CRC 数据暂存起来
    // 整个安全字不参与CRC的计算
    tmp = message[--Bytelength];
    --Bytelength;

    // For all the bytes of the message.
    for ( Byteidx = 0U; Byteidx < Bytelength; Byteidx++ )
    {
        // “crc” is calculated as the XOR operation from the previous “crc” and the “message”. “^” is the XOR operator.
        crc ^= message[Byteidx];
        // For each bit position in a 8-bit word
        for ( Bitidx = 0U; Bitidx < 8U; Bitidx++ )
        {
            // If the MSB of the “crc” is 1(with the &0x80 mask we get the MSB of the crc).
            if (( crc & 0x80U ) != 0U )
            {
                // “crc” advances on position 
                // (“crc” is moved left 1 bit: the MSB is deleted since it will be cancelled out with the first one
                // of the generator polynomial and a new bit from the “message” is taken as LSB.)
                crc <<= 1U;
                // “crc” is calculated as the XOR operation from the previous “crc” and the generator
                // polynomial (0x1D for TLE5012B). Be aware that here the x8 bit is not taken since
                // the MSB of the “crc” already has been deleted in the previous step.
                crc ^= 0x1DU;
            }
            // In case the crc MSB is 0.
            else
                // “crc” advances one position (this step is to ensure that the XOR operation is only
                // done when the generator polynomial is aligned with a MSB of the message that is “1”.
                crc <<= 1;
        }
    }
    // Return the inverted “crc” remainder(“~” is the invertion operator).
    // An alternative to the “~” operator would be a XOR operation between “crc” and a 0xFF polynomial.
    return (( ~crc ) & 0xFFU ) != tmp;
}

/**
  * @brief  从给定的数据地址开始, 连续读取多个数据
  *         警告, 多线程同时调用会出现严重的问题, 同时与 "TLE5012_write_reg()"函数冲突
  * @param  p_tle_dev   指向磁编组件的实例
  * @param  target_reg  磁编寄存器地址
  * @param  p_buffer    数据收发缓存区
  * @param  length      需要操作的数据长度, 不包含控制字和安全字长度. (4U)
  * @retval 是否存在错误
  */
TLE5012_ERROR_t TLE5012_read_reg( TLE5012_Handle_t * p_tle_dev, TLE5012_CMD_READ_t target_reg, TLE5012_DATA_t * p_buffer, uint16_t length )
{
    // 配置控制字
    uint16_t cmd = target_reg;              // 设置控制字的基础, 此基础确认了具体的读写, LOCK的值, 寄存器地址
    // cmd |= TLE_CMD_UPD_BUFFER;               // 确认是否缓冲区的值   // 屏蔽后不使用
    cmd |= length;                          // 需要传输的数据长度，不包含安全字. (4U)

    // 整理数据缓冲区
    p_buffer->send_data[0U] = cmd;          // 储存需要发送的控制字
    p_buffer->send_len = 1U;                // 因为是需要读取数据, 需要发送的只有一个控制字
    p_buffer->recv_len = length + 1U;       // 一共需要读取的数据长度, 包含安全字. (5U)

    // 交换数据, 并判断数据交换时是否发生错误, 有错误进入
    uint16_t ret = p_tle_dev->SSC_send_and_recv( p_buffer->send_data, p_buffer->recv_data, p_buffer->send_len, p_buffer->recv_len );
    if ( TLE5012_ERROR_OK != ret )
    {
        p_tle_dev->error_code |= TLE5012_ERROR_API;
        return TLE5012_ERROR_NOK;
    }

    // 计算CRC并判断CRC是否一致, 不一致则报错退出
    // 查表方式, 两种方法选其一. (4U)
    if ( CRC8_TABLE( p_buffer, TableCRC ))
    {
        p_tle_dev->error_code |= TLE5012_ERROR_CRC;
        return TLE5012_ERROR_NOK;
    }

    // 确认安全字, 数组中数据长度与下标相差1. (4U)
    uint16_t safety = p_buffer->recv_data[p_buffer->recv_len - 1U]; // 取出接收到的安全字
    safety &= TLE_SFT_SATA_BIT_MASK;                                // 屏蔽无效bit

    // 判断安全字中是否存在错误状态
    if ( _IS_TLE_SFT_ERROR( safety ))
    {
        p_tle_dev->error_code |= ~safety;                           // 记录错误状态
        return TLE5012_ERROR_NOK;
    }

    // 配置控制字并整理 -> 交换数据 -> 计算CRC -> 错误判断 -> 一切正常返回
    return TLE5012_ERROR_OK;
}

/**
  * @brief  从给定的数据地址开始, 连续写入多个数据
  *         警告, 多线程同时调用会出现严重的问题, 同时与 "TLE5012_read_reg()"函数冲突
  * @param  p_tle_dev   指向磁编组件的实例
  * @param  target_reg  磁编寄存器地址
  * @param  p_buffer    需要发送或接收的数据指针
  * @param  length      需要操作的数据长度, 不包含控制字和安全字长度. (4U)
  * @retval 是否存在错误
  */
TLE5012_ERROR_t TLE5012_write_reg( TLE5012_Handle_t * p_tle_dev, TLE5012_CMD_WRITE_t target_reg, TLE5012_DATA_t * p_buffer, uint16_t length )
{

    // 配置控制字
    uint16_t cmd = target_reg;          // 设置控制字的基础, 此基础确认了具体的读写, LOCK的值, 寄存器地址
    // cmd |= TLE_CMD_UPD_BUFFER;          // 确认是否缓冲区的值   // 屏蔽后不使用
    cmd |= length;                      // 需要传输的数据长度，不包含安全字. (4U)

    // 整理数据缓冲区
    uint16_t tmp[14U];
    memcpy( tmp, p_buffer->send_data, ( length * 2U ));             // 为了发送数据, 数据转移到跳板
    p_buffer->send_data[0U] = cmd;                                  // 储存需要发送的控制字
    memcpy(( p_buffer->send_data + 1U ), tmp, ( length * 2U ));     // 将数据从跳板搬运回发送区
    p_buffer->send_len = length + 1U;                               // 需要发送的实际数据长度, 加一个控制字. (5U)
    p_buffer->recv_len = 1U;                                        // 需要接收1个安全字

    // 交换数据, 并判断数据交换时是否发生错误, 有错误进入
    uint16_t ret = p_tle_dev->SSC_send_and_recv( p_buffer->send_data, p_buffer->recv_data, p_buffer->send_len, p_buffer->recv_len );
    if ( TLE5012_ERROR_OK != ret )
    {
        p_tle_dev->error_code |= TLE5012_ERROR_API;
        return TLE5012_ERROR_NOK;
    }

    // 计算CRC并判断CRC是否一致, 不一致则报错退出
    // 查表方式, 两种方法选其一. (4U)
    if ( CRC8_TABLE( p_buffer, TableCRC ))
    {
        p_tle_dev->error_code |= TLE5012_ERROR_CRC;
        return TLE5012_ERROR_NOK;
    }

    // 确认安全字, 数组中数据长度与下标相差1. (4U)
    uint16_t safety = p_buffer->recv_data[p_buffer->recv_len - 1U]; // 取出接收到的安全字
    safety &= TLE_SFT_SATA_BIT_MASK;                                // 屏蔽无效bit

    // 判断安全字中是否存在错误状态
    if ( _IS_TLE_SFT_ERROR( safety ))
    {
        p_tle_dev->error_code |= ~safety;                           // 记录错误状态
        return TLE5012_ERROR_NOK;
    }

    // 配置控制字并整理 -> 交换数据 -> 计算CRC -> 错误判断 -> 一切正常返回
    return TLE5012_ERROR_OK;
}

/**
  * @brief  TLE5012芯片的初始化函数
  * @param  p_tle_dev   指向磁编组件的实例
  *         初始化步骤
  *         step0: 读取整个片内寄存器的实时值，并保存
  *         step1: 确认ACSTAT寄存器的默认值
  *         step2: 确认派生器件类型 (E1000)
  *         step3: 设置旋转增量方向
  *         step4: 设置绝对零角度
  *         step5: 设置adcy寄存器中CRC相关的数据 (忽略, 因为芯片开启了自动校准)
  *         step6: 储存最终芯片内配置及数据, 与step0完全一样
  */
void TLE5012_Init( TLE5012_Handle_t * p_tle_dev )
{
    // step0: 读取整个片内寄存器的实时值，并保存
    if ( TLE5012_ERROR_OK != TLE5012_ic_reg_backups( p_tle_dev ))
    {
        p_tle_dev->error_code |= TLE5012_ERROR_BACKUP;              // 磁编初始化失败
        Error_Handler();                                            // 进入死机状态
    }

    // step1: 确认ACSTAT寄存器的默认值
    if (( TLE5012_DEFAULT_ACSTAT1 != p_tle_dev->reg.acstat ) &&    // 确认芯片ACSTAT寄存器的默认配置
        ( TLE5012_DEFAULT_ACSTAT2 != p_tle_dev->reg.acstat ))
    {
        p_tle_dev->error_code |= TLE5012_ERROR_IC;                  // 磁编初始化失败
        Error_Handler();                                            // 进入死机状态
    }

    // step2: 目标芯片为TLE5012B-E1000
    /* if (( TLE5012_DEFAULT_MOD1 != ( TLE5012_DEFAULT_MOD1_MASK & p_tle_dev->reg.mod1 )) ||    // 确认芯片MOD_1寄存器的默认配置
           ( TLE5012_DEFAULT_MOD2 != ( TLE5012_DEFAULT_MOD2_MASK & p_tle_dev->reg.mod2 )) ||    // 确认芯片MOD_2寄存器的默认配置
           ( TLE5012_DEFAULT_MOD3 != ( TLE5012_DEFAULT_MOD3_MASK & p_tle_dev->reg.mod3 )) ||    // 确认芯片MOD_3寄存器的默认配置
           ( TLE5012_DEFAULT_IFAB != ( TLE5012_DEFAULT_IFAB_MASK & p_tle_dev->reg.ifab )) ||    // 确认芯片IFAB寄存器的默认配置
           ( TLE5012_DEFAULT_MOD4 != ( TLE5012_DEFAULT_MOD4_MASK & p_tle_dev->reg.mod4 )) ||    // 确认芯片MOD_4寄存器的默认配置
           ( TLE5012_DEFAULT_TCO_Y != ( TLE5012_DEFAULT_TCO_Y_MASK & p_tle_dev->reg.tcoy )))    // 确认芯片TCO_Y寄存器的默认配置
    {
        p_tle_dev->error_code |= TLE5012_ERROR_INIT;        // 磁编初始化失败
        Error_Handler();                                    // 进入死机状态
    } */

    // step3: 设置旋转增量方向
    if ( TLE5012_ERROR_OK != TLE5012_set_dir( p_tle_dev ))        // 确认芯片方向是否与需要设置的方向一致, 不一致则调整下
    {
        p_tle_dev->error_code |= TLE5012_ERROR_DIR;                 // 磁编初始化失败
        Error_Handler();                                            // 进入死机状态
    }

    // step4: 设置绝对零角度
    // 读取FLASH中保存的绝对零角度

    // 写入
    if ( TLE5012_ERROR_OK != TLE5012_set_zero_position( p_tle_dev ))
    {
        p_tle_dev->error_code |= TLE5012_ERROR_ZERO;                // 磁编初始化失败
        Error_Handler();                                            // 进入死机状态
    }

    // step5: 设置adcy寄存器中CRC相关的数据 (忽略, 因为芯片开启了自动校准)

    // step6: 储存最终芯片内配置及数据
    if ( TLE5012_ERROR_OK != TLE5012_ic_reg_backups( p_tle_dev ))
    {
        p_tle_dev->error_code |= TLE5012_ERROR_BACKUP;              // 磁编初始化失败
        Error_Handler();                                            // 进入死机状态
    }

    /* 提高SPI通信速度 */
    // New_SPI2_Init();

    // 初始化完成
}

/**
  * @brief  备份整个磁编的内部REG内容
  * @param  p_tle_dev   指向磁编组件的实例
  * @retval 是否存在错误
  */
TLE5012_ERROR_t TLE5012_ic_reg_backups( TLE5012_Handle_t * p_tle_dev )
{
    TLE5012_DATA_t temp_buffer;                     // 定义收发数据的数据缓冲区
    volatile uint16_t *temp_read = temp_buffer.recv_data;    // 映射实例内的接收缓冲区地址
    volatile uint8_t target_idx;

    // 从0x00地址开始, 5个数据
    if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_STAT, &temp_buffer, 5U ))
        return TLE5012_ERROR_NOK;
    target_idx = 0U;
    p_tle_dev->reg.stat     = *( temp_read + target_idx++ );    // 0x00U
    p_tle_dev->reg.acstat   = *( temp_read + target_idx++ );    // 0x01U
    p_tle_dev->reg.aval     = *( temp_read + target_idx++ );    // 0x02U
    p_tle_dev->reg.aspd     = *( temp_read + target_idx++ );    // 0x03U
    p_tle_dev->reg.arev     = *( temp_read + target_idx++ );    // 0x04U

    // 从0x05地址开始, 13个数据
    if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_FSYNC, &temp_buffer, 13U ))
        return TLE5012_ERROR_NOK;
    target_idx = 0U;
    p_tle_dev->reg.fsync    = *( temp_read + target_idx++ );    // 0x05U
    p_tle_dev->reg.mod1     = *( temp_read + target_idx++ );    // 0x06U
    p_tle_dev->reg.sil      = *( temp_read + target_idx++ );    // 0x07U
    p_tle_dev->reg.mod2     = *( temp_read + target_idx++ );    // 0x08U
    p_tle_dev->reg.mod3     = *( temp_read + target_idx++ );    // 0x09U
    p_tle_dev->reg.offx     = *( temp_read + target_idx++ );    // 0x0AU
    p_tle_dev->reg.offy     = *( temp_read + target_idx++ );    // 0x0BU
    p_tle_dev->reg.synch    = *( temp_read + target_idx++ );    // 0x0CU
    p_tle_dev->reg.ifab     = *( temp_read + target_idx++ );    // 0x0DU
    p_tle_dev->reg.mod4     = *( temp_read + target_idx++ );    // 0x0EU
    p_tle_dev->reg.tcoy     = *( temp_read + target_idx++ );    // 0x0FU
    p_tle_dev->reg.adcx     = *( temp_read + target_idx++ );    // 0x10U
    p_tle_dev->reg.adcy     = *( temp_read + target_idx++ );    // 0x11U

    // 从0x14地址开始, 2个数据
    if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_D_MAG, &temp_buffer, 2U ))
        return TLE5012_ERROR_NOK;
    target_idx = 0U;
    p_tle_dev->reg.dmag     = *( temp_read + target_idx++ );    // 0x14U
    p_tle_dev->reg.traw     = *( temp_read + target_idx++ );    // 0x15U

    // 从0x20地址开始, 1个数据
    if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_IIF_CNT, &temp_buffer, 1U ))
        return TLE5012_ERROR_NOK;
    target_idx = 0U;
    p_tle_dev->reg.iifcnt   = *( temp_read + target_idx++ );    // 0x20U

    // 从0x30地址开始, 1个数据
    if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_T25O, &temp_buffer, 1U ))
        return TLE5012_ERROR_NOK;
    target_idx = 0U;
    p_tle_dev->reg.t25o     = *( temp_read + target_idx++ );    // 0x30U

    return TLE5012_ERROR_OK;
}

/**
  * @brief  设置TLE5012芯片的方向, 手册推荐做法
  * @param  p_tle_dev   指向磁编组件的实例
  * @retval 是否存在错误
  *         设置流程如下
  *         step0: 判断芯片方向和需要设置的方向是否一致
  *         step1: 获取MOD_2默认值, 确认需要设置的方向   (0x0801)
  *         step2: 写MOD_2寄存器改变方向并关闭自动校准   (0x0808)
  *         step3: 等待128us或更长时间
  *         step4: 开启自动校正                         (0x0809)
  *         step5: 等待128us或更长时间
  *         step6: 读取STAT寄存器, 在设置过程中此寄存器可能会报错, 需要清除一次
  *         step7: 再次读取, 判断本次读取是否还有错误
  */
TLE5012_ERROR_t TLE5012_set_dir( TLE5012_Handle_t * p_tle_dev )
{
    uint16_t expected_dir = p_tle_dev->dir;     // 确认期望方向
    uint16_t set_value = p_tle_dev->reg.mod2;   // 保护需要的寄存器值
    TLE5012_DATA_t temp_buffer;                 // 定义收发数据的数据缓冲区

    // step0: 对读取到的MOD_2寄存器做相应的位屏蔽, 并确认与期望方向是否一致
    if ( expected_dir != ( set_value & 0x08U ))
    {
        // step1: 获取MOD_2默认值, 确认需要设置的方向. (0x0801)
        set_value = ( set_value & 0xFFF7U ) | expected_dir;     // 第3bit, 先清零, 后设置方向

        // step2: 写MOD_2寄存器改变方向并关闭自动校准. (0x0808)
        temp_buffer.send_data[0U] = set_value & 0xFFFCU;        // 给0~1bit设置0, 关闭自动校准, 赋值
        if ( TLE5012_ERROR_OK != TLE5012_write_reg( p_tle_dev, SSC_WRITE_MOD_2, &temp_buffer, 1U ))
            return TLE5012_ERROR_DIR;

        // step3: 等待128us或更长时间
        HAL_Delay(2U);

        // step4: 开启自动校正. (0x0809)
        // 因为之前关闭过自动校正, 所以本次返回的安全字中会有错误位
        temp_buffer.send_data[0U] = set_value;                  // 带自动校准, 赋值
        TLE5012_write_reg( p_tle_dev, SSC_WRITE_MOD_2, &temp_buffer, 1U );
        p_tle_dev->error_code &= ~TLE_SFT_SATA_BIT_MASK_SYS;    // 清除需要忽略的错误位
        if ( TLE5012_ERROR_OK != p_tle_dev->error_code )        // 再次判断是否存在其他错误
            return TLE5012_ERROR_DIR; 

        // step5: 等待128us或更长时间
        HAL_Delay(2U);

        // step6: 读取STAT寄存器, 在设置过程中此寄存器可能会报错, 需要清除一次
        TLE5012_read_reg( p_tle_dev, SSC_READ_STAT, &temp_buffer, 1U );
        p_tle_dev->error_code &= ~TLE_SFT_SATA_BIT_MASK_SYS;    // 清除需要忽略的错误位
        if ( TLE5012_ERROR_OK != p_tle_dev->error_code )        // 再次判断是否存在其他错误
            return TLE5012_ERROR_DIR; 

        // step7: 再次读取, 判断本次读取是否还有错误
        if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_STAT, &temp_buffer, 1U ))
            return TLE5012_ERROR_DIR;
    }

    // 设置方向完成
    return TLE5012_ERROR_OK;
}
/**
  * @brief  设置编码器的绝对零角度
  * @param  p_tle_dev   指向磁编组件的实例
  * @retval 是否存在错误
  *         设置流程如下
  *         step1: 确认补偿标志位, 有标志则执行补偿任务
  *         step2: 确认补偿方向标志位, 与当前的芯片方向一致则执行补偿任务，否则返回错误
  *         step3: 读取相关寄存器
  *         step4: 合并数据, 保留需要的, 写入补偿值
  *         step5: 读取验证
  */
TLE5012_ERROR_t TLE5012_set_zero_position( TLE5012_Handle_t * p_tle_dev )
{
    TLE5012_DATA_t temp_buffer;                     // 定义收发数据的数据缓冲区

    p_tle_dev->set_zero_ok = false;

    // step1: 确认补偿标志位, 有标志则执行补偿任务
    if ( TLE_ZERO_POSITION_MARK == ( p_tle_dev->zero & TLE_ZERO_POSITION_MARK )) 
    {
        // step2: 确认补偿方向标志位, 与当前的芯片方向一致
        if ( p_tle_dev->dir == ( p_tle_dev->zero & TLE_ZERO_POSITION_DIR_MARK ))
        {
            // step3: 读取相关寄存器, 从新读取是因为有可能在别处重新多次调用, 不容易出错
            if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_MOD_3, &temp_buffer, 1U ))
                return TLE5012_ERROR_ZERO;

            // step4: 合并数据, 保留需要的, 写入补偿值
            uint16_t ang_base = ( temp_buffer.recv_data[0U] & 0x000FU ) | ( p_tle_dev->zero & 0xFFF0U );
            temp_buffer.send_data[0] = ang_base;
            if ( TLE5012_ERROR_OK != TLE5012_write_reg( p_tle_dev, SSC_WRITE_MOD_3, &temp_buffer, 1U ))
                return TLE5012_ERROR_ZERO;

            // step5: 读取验证
            if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_MOD_3, &temp_buffer, 1U ))
                return TLE5012_ERROR_ZERO;
            if ( ang_base != temp_buffer.recv_data[0U] )
                return TLE5012_ERROR_ZERO;

            // 完成, 设置绝对零点设置成功
            p_tle_dev->set_zero_ok = true;
        }
    }

    // 完成
    return TLE5012_ERROR_OK;
}

/**
  * @brief  获取编码器的绝对零角偏移量
  * @param  p_tle_dev   指向磁编组件的实例
  * @retval 是否存在错误
  *         设置流程如下
  *         step1: 读取当前角度值和角度补偿量
  *         step2: 对补偿量做数据处理
  *         step3: 确认正负补偿值
  *         step4: 设置标志位
  */
TLE5012_ERROR_t TLE5012_git_zero_position( TLE5012_Handle_t * p_tle_dev )
{
    uint16_t ang_value, ang_base;
    TLE5012_DATA_t temp_buffer;                 // 定义收发数据的数据缓冲区

    // 读取当前角度值和默认校准角度值
    if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_AVAL, &temp_buffer, 1U ))
        return TLE5012_ERROR_ZERO;
    ang_value = temp_buffer.recv_data[0U];

    if ( TLE5012_ERROR_OK != TLE5012_read_reg( p_tle_dev, SSC_READ_MOD_3, &temp_buffer, 1U ))
        return TLE5012_ERROR_ZERO;
    ang_base = temp_buffer.recv_data[0U];

    // 对读取到的数据做处理
    ang_value = ( ang_value & 0x7FFFU ) >> 3U;  // 角度值去除最高位, 并降低3位分辨率
    ang_base >>= 4U;                            // 去除无用位

    // 判断正、负补偿
    // 不相等需要正补偿, 相等需要负补偿, 补偿精度12bit, 存放在15:4位中
    if ( 0U == p_tle_dev->dir )
    {
        p_tle_dev->zero = ( ang_base - ang_value) << 4U;
    }
    else
    {
        p_tle_dev->zero = ( ang_base + ang_value) << 4U;
        p_tle_dev->zero |= TLE_ZERO_POSITION_DIR_MARK;
    }
    
    // 设置标志位, 第0bit为标志位
    p_tle_dev->zero |= TLE_ZERO_POSITION_MARK;

    // 完成
    return TLE5012_ERROR_OK;    
}

/************************************************************/
/*************** 基于 HAL 库实现配置校验, 结尾 ***************/
/***********************************************************/
