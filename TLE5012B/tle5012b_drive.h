/*
 * tle5012b_drive.h
 *
 *  Created on: 2021年6月19日
 *      Author: EE-Sen-Matebook
 */

#ifndef _TLE5012B_H_
#define _TLE5012B_H_

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>

/* TLE5012的精度 */
#define TLE5012_SSC_PRECISION           ((uint16_t)32768U)  /* 2^15 */

/* TLE5012芯片初始化时, ACSTAT寄存器(0x01), 上电时的默认值0x58EE或0x18EE */
#define TLE5012_DEFAULT_ACSTAT1         0x58EEU
#define TLE5012_DEFAULT_ACSTAT2         0x18EEU

/* 请确认你使用的器件具体型号 */
#define TLE_E1000
/* #define TLE_E3005
#define TLE_E5000
#define TLE_E5020
#define TLE_E9000 */

#define TLE5012_DEFAULT_MOD1_MASK       0xC017U     // 0x06, 1100 0000 0001 0111, 使用与逻辑屏蔽, 默认配置位用1填充, x位用0填充
#define TLE5012_DEFAULT_MOD2_MASK       0x7FFFU     // 0x08, 0111 1111 1111 1111, 使用与逻辑屏蔽, 默认配置位用1填充, x位用0填充
#define TLE5012_DEFAULT_MOD3_MASK       0x000FU     // 0x09, 0000 0000 0000 1111, 使用与逻辑屏蔽, 默认配置位用1填充, x位用0填充
#define TLE5012_DEFAULT_IFAB_MASK       0x000FU     // 0x0D, 0000 0000 0000 1111, 使用与逻辑屏蔽, 默认配置位用1填充, x位用0填充
#define TLE5012_DEFAULT_MOD4_MASK       0x01FBU     // 0x0E, 0000 0001 1111 1011, 使用与逻辑屏蔽, 默认配置位用1填充, x位用0填充
#define TLE5012_DEFAULT_TCO_Y_MASK      0x0010U     // 0x0F, 0000 0001 0000 0000, 使用与逻辑屏蔽, 默认配置位用1填充, x位用0填充

#ifdef TLE_E1000
/* 基于E1000器件初始化的默认值 */
#define TLE5012_DEFAULT_MOD1            0x4001U     // 0x06, 01xx xxxx xxx0 x001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD2            0x0801U     // 0x08, x000 1000 0000 0001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD3            0x0000U     // 0x09, xxxx xxxx xxxx 0000, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_IFAB            0x000BU     // 0x0D, xxxx xxxx xxxx 1011, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD4            0x0020U     // 0x0E, xxxx xxx0 0010 0x00, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_TCO_Y           0x0100U     // 0x0F, xxxx xxx1 xxxx xxxx, 默认配置位保留原有, x位用0填充
#endif /* TLE_E1000 */

#ifdef TLE_E3005
/* 基于E3005器件初始化的默认值(未确认) */
#define TLE5012_DEFAULT_MOD1            0x4001U     // 0x06, 01xx xxxx xxx0 x001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD2            0x0801U     // 0x08, x000 1000 0000 0001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD3            0x0000U     // 0x09, xxxx xxxx xxxx 0000, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_IFAB            0x000BU     // 0x0D, xxxx xxxx xxxx 1011, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD4            0x0020U     // 0x0E, xxxx xxx0 0010 0x00, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_TCO_Y           0x0100U     // 0x0F, xxxx xxx1 xxxx xxxx, 默认配置位保留原有, x位用0填充
#endif /* TLE_E3005 */

#ifdef TLE_E5000
/* 基于E5000器件初始化的默认值(未确认) */
#define TLE5012_DEFAULT_MOD1            0x4001U     // 0x06, 01xx xxxx xxx0 x001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD2            0x0801U     // 0x08, x000 1000 0000 0001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD3            0x0000U     // 0x09, xxxx xxxx xxxx 0000, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_IFAB            0x000BU     // 0x0D, xxxx xxxx xxxx 1011, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD4            0x0020U     // 0x0E, xxxx xxx0 0010 0x00, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_TCO_Y           0x0100U     // 0x0F, xxxx xxx1 xxxx xxxx, 默认配置位保留原有, x位用0填充
#endif /* TLE_E5000 */

#ifdef TLE_E5020
/* 基于E5020器件初始化的默认值(未确认) */
#define TLE5012_DEFAULT_MOD1            0x4001U     // 0x06, 01xx xxxx xxx0 x001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD2            0x0801U     // 0x08, x000 1000 0000 0001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD3            0x0000U     // 0x09, xxxx xxxx xxxx 0000, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_IFAB            0x000BU     // 0x0D, xxxx xxxx xxxx 1011, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD4            0x0020U     // 0x0E, xxxx xxx0 0010 0x00, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_TCO_Y           0x0100U     // 0x0F, xxxx xxx1 xxxx xxxx, 默认配置位保留原有, x位用0填充
#endif /* TLE_E5020 */

#ifdef TLE_E9000
/* 基于E9000器件初始化的默认值(未确认) */
#define TLE5012_DEFAULT_MOD1            0x4001U     // 0x06, 01xx xxxx xxx0 x001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD2            0x0801U     // 0x08, x000 1000 0000 0001, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD3            0x0000U     // 0x09, xxxx xxxx xxxx 0000, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_IFAB            0x000BU     // 0x0D, xxxx xxxx xxxx 1011, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_MOD4            0x0020U     // 0x0E, xxxx xxx0 0010 0x00, 默认配置位保留原有, x位用0填充
#define TLE5012_DEFAULT_TCO_Y           0x0100U     // 0x0F, xxxx xxx1 xxxx xxxx, 默认配置位保留原有, x位用0填充
#endif /* TLE_E9000 */

/* 控制字中, 各配置的功能寄存器的地址偏移 */
#define TLE_CMD_IO_DIR_BITS             15U
#define TLE_CMD_LOCK_SHIFT_BITS         11U
#define TLE_CMD_UPD_SHIFT_BITS          10U
#define TLE_CMD_ADDR_SHIFT_BITS         4U
#define TLE_CMD_ND_BITS                 0U
/* 安全字中, 各状态位所在的位置 */
#define TLE_SFT_SATA_BIT_MASK           0xF000U
#define TLE_SFT_SATA_BIT_MASK_RST       0x8000U
#define TLE_SFT_SATA_BIT_MASK_SYS       0x4000U
#define TLE_SFT_SATA_BIT_MASK_ADDR      0x2000U
#define TLE_SFT_SATA_BIT_MASK_INVAL     0x1000U
/* 机械绝对0角度标志位 */
#define TLE_ZERO_POSITION_MARK          0x0001U
#define TLE_ZERO_POSITION_DIR_MARK      0x0008U
/* 切换为操作更新缓冲区 UPD = 1 */
#define TLE_CMD_UPD_BUFFER              0x0400U      // CMD 10bit 读取更新缓冲区值

/* 安全字中, 判断是否有错误状态 */
#define _IS_TLE_SFT_ERROR( __STAT__ )   ( __STAT__ != TLE_SFT_SATA_BIT_MASK)        // 等式成立，说明存在错误

/* 使用0x04中的内容计算速度时需要用到定义 */
#define TLE_UPDATE_RATE                 427U         // MOD_1寄存器中FIR_MD的值为 01b 时内部的更新速率, 单位( us * 10U )


/**
  * @brief  枚举读取控制字集合
  * @brief  默认包含: 读, LOCK值, 实时寄存器值, 寄存器地址, 数据长度(不含安全字)
  * @brief  默认包含: 读, LOCK值, 实时寄存器值, 寄存器地址, 数据长度(不含安全字)
  */
typedef enum
{
    SSC_READ_STAT       = 0x8000U,  // 1_000 0_0_00 0000 _0000  // STAT status register
    SSC_READ_ACSTAT     = 0x8010U,  // 1_000 0_0_00 0001 _0000  // ACSTAT activation status register
    SSC_READ_AVAL       = 0x8020U,  // 1_000 0_0_00 0010 _0000  // AVAL angle value register
    SSC_READ_ASPD       = 0x8030U,  // 1_000 0_0_00 0011 _0000  // ASPD angle speed register
    SSC_READ_AREV       = 0x8040U,  // 1_000 0_0_00 0100 _0000  // AREV angle revolution register
    SSC_READ_FSYNC      = 0xD050U,  // 1_101 0_0_00 0101 _0000  // FSYNC frame synchronization register
    SSC_READ_MOD_1      = 0xD060U,  // 1_101 0_0_00 0110 _0000  // MOD_1 interface mode1 register
    SSC_READ_SIL        = 0xD070U,  // 1_101 0_0_00 0111 _0000  // SIL register
    SSC_READ_MOD_2      = 0xD080U,  // 1_101 0_0_00 1000 _0000  // MOD_2 interface mode2 register
    SSC_READ_MOD_3      = 0xD090U,  // 1_101 0_0_00 1001 _0000  // MOD_3 interface mode3 register
    SSC_READ_OFFX       = 0xD0A0U,  // 1_101 0_0_00 1010 _0000  // OFFX offset x
    SSC_READ_OFFY       = 0xD0B0U,  // 1_101 0_0_00 1011 _0000  // OFFY offset y
    SSC_READ_SYNCH      = 0xD0C0U,  // 1_101 0_0_00 1100 _0000  // SYNCH synchronicity
    SSC_READ_IFAB       = 0xD0D0U,  // 1_101 0_0_00 1101 _0000  // IFAB register
    SSC_READ_MOD_4      = 0xD0E0U,  // 1_101 0_0_00 1110 _0000  // MOD_4 interface mode4 register
    SSC_READ_TCO_Y      = 0xD0F0U,  // 1_101 0_0_00 1111 _0000  // TCO_Y temperature coefficient register
    SSC_READ_ADC_X      = 0xD100U,  // 1_101 0_0_01 0000 _0000  // ADC_X ADC X-raw value
    SSC_READ_ADC_Y      = 0xD110U,  // 1_101 0_0_01 0001 _0000  // ADC_Y ADC Y-raw value
    SSC_READ_D_MAG      = 0x8140U,  // 1_000 0_0_01 0100 _0000  // D_MAG angle vector magnitude
    SSC_READ_T_RAW      = 0x8150U,  // 1_000 0_0_01 0101 _0000  // T_RAW temperature sensor raw-value
    SSC_READ_IIF_CNT    = 0x8200U,  // 1_000 0_0_10 0000 _0000  // IIF_CNT IIF counter value
    SSC_READ_T25O       = 0x8300U,  // 1_000 0_0_11 0000 _0000  // T25O temperature 25°c offset value
} TLE5012_CMD_READ_t;

/**
  * @brief  枚举写入控制字集合
  * @brief  默认包含: 写, LOCK值, 实时寄存器值, 寄存器地址, 数据长度(不含安全字)
  * @brief  默认包含: 写, LOCK值, 实时寄存器值, 寄存器地址, 数据长度(不含安全字)
  */
typedef enum
{
    SSC_WRITE_STAT      = 0x0000U,  // 0_000 0_0_00 0000 _0000  // STAT status register
    SSC_WRITE_ACSTAT    = 0x0010U,  // 0_000 0_0_00 0001 _0000  // ACSTAT activation status register
    SSC_WRITE_AVAL      = 0x0020U,  // 0_000 0_0_00 0010 _0000  // AVAL angle value register
    SSC_WRITE_ASPD      = 0x0030U,  // 0_000 0_0_00 0011 _0000  // ASPD angle speed register
    SSC_WRITE_AREV      = 0x0040U,  // 0_000 0_0_00 0100 _0000  // AREV angle revolution register
    SSC_WRITE_FSYNC     = 0x5050U,  // 0_101 0_0_00 0101 _0000  // FSYNC frame synchronization register
    SSC_WRITE_MOD_1     = 0x5060U,  // 0_101 0_0_00 0110 _0000  // MOD_1 interface mode1 register
    SSC_WRITE_SIL       = 0x5070U,  // 0_101 0_0_00 0111 _0000  // SIL register
    SSC_WRITE_MOD_2     = 0x5080U,  // 0_101 0_0_00 1000 _0000  // MOD_2 interface mode2 register
    SSC_WRITE_MOD_3     = 0x5090U,  // 0_101 0_0_00 1001 _0000  // MOD_3 interface mode3 register
    SSC_WRITE_OFFX      = 0x50A0U,  // 0_101 0_0_00 1010 _0000  // OFFX offset x
    SSC_WRITE_OFFY      = 0x50B0U,  // 0_101 0_0_00 1011 _0000  // OFFY offset y
    SSC_WRITE_SYNCH     = 0x50C0U,  // 0_101 0_0_00 1100 _0000  // SYNCH synchronicity
    SSC_WRITE_IFAB      = 0x50D0U,  // 0_101 0_0_00 1101 _0000  // IFAB register
    SSC_WRITE_MOD_4     = 0x50E0U,  // 0_101 0_0_00 1110 _0000  // MOD_4 interface mode4 register
    SSC_WRITE_TCO_Y     = 0x50F0U,  // 0_101 0_0_00 1111 _0000  // TCO_Y temperature coefficient register
    SSC_WRITE_ADC_X     = 0x5100U,  // 0_101 0_0_01 0000 _0000  // ADC_X ADC X-raw value
    SSC_WRITE_ADC_Y     = 0x5110U,  // 0_101 0_0_01 0001 _0000  // ADC_Y ADC Y-raw value
    SSC_WRITE_D_MAG     = 0x0140U,  // 0_000 0_0_01 0100 _0000  // D_MAG angle vector magnitude
    SSC_WRITE_T_RAW     = 0x0150U,  // 0_000 0_0_01 0101 _0000  // T_RAW temperature sensor raw-value
    SSC_WRITE_IIF_CNT   = 0x0200U,  // 0_000 0_0_10 0000 _0000  // IIF_CNT IIF counter value
    SSC_WRITE_T25O      = 0x0300U,  // 0_000 0_0_11 0000 _0000  // T25O temperature 25°c offset value
} TLE5012_CMD_WRITE_t;

/**
  * @brief  枚举电机旋转方向
  * @brief  枚举电机旋转方向
  * @brief  枚举电机旋转方向
  */
typedef enum
{
    CW                      = 0U,   // 逆时针旋转为正方向
    CCW                     = 0x08U // 顺时针旋转为正方向
} TLE5012_DIR_t;

/**
  * @brief  磁编内部寄存器的映射
  * @brief  磁编内部寄存器的映射
  * @brief  磁编内部寄存器的映射
  */
typedef struct
{
    uint16_t stat;                  // 0x00U
    uint16_t acstat;                // 0x01U
    uint16_t aval;                  // 0x02U
    uint16_t aspd;                  // 0x03U
    uint16_t arev;                  // 0x04U
    uint16_t fsync;                 // 0x05U
    uint16_t mod1;                  // 0x06U
    uint16_t sil;                   // 0x07U
    uint16_t mod2;                  // 0x08U
    uint16_t mod3;                  // 0x09U
    uint16_t offx;                  // 0x0AU
    uint16_t offy;                  // 0x0BU
    uint16_t synch;                 // 0x0CU
    uint16_t ifab;                  // 0x0DU
    uint16_t mod4;                  // 0x0EU
    uint16_t tcoy;                  // 0x0FU
    uint16_t adcx;                  // 0x10U
    uint16_t adcy;                  // 0x11U
    uint16_t dmag;                  // 0x14U
    uint16_t traw;                  // 0x15U
    uint16_t iifcnt;                // 0x20U
    uint16_t t25o;                  // 0x30U
    uint16_t safety;                // 储存最后一次读取或写入的安全字
} TLE5012_REG_t;

/**
  * @brief  枚举各种错误状态
  * @brief  枚举各种错误状态
  * @brief  枚举各种错误状态
  */
typedef enum
{
    TLE5012_ERROR_OK        = 0U,       // 正常
    TLE5012_ERROR_NOK       = 0x0001U,  // 非正常
    TLE5012_ERROR_BACKUP    = 0x0010U,  // 备份失败
    TLE5012_ERROR_IC        = 0x0020U,  // 使用的芯片派生错误
    TLE5012_ERROR_DIR       = 0x0040U,  // 初始化错误
    TLE5012_ERROR_ZERO      = 0x0080U,  // 机械绝对零点偏移输入错误
    TLE5012_ERROR_API       = 0x0400U,  // HAL库的API错误
    TLE5012_ERROR_CRC       = 0x0800U,  // CRC校验错误
    TLE5012_ERROR_IC_INVAL  = 0x1000U,  // 说明磁铁或PCBA安装存在异常
    TLE5012_ERROR_IC_ADDR   = 0x2000U,  // 输入的寄存器地址错误
    TLE5012_ERROR_IC_SYS    = 0x4000U,  // 芯片内部错误
    TLE5012_ERROR_IC_RST    = 0x8000U   // 芯片发生过复位
} TLE5012_ERROR_t;

/**
  * @brief  定义类型, 使用HAL库读写磁编数据时的收发缓冲区
  * @brief  定义类型, 使用HAL库读写磁编数据时的收发缓冲区
  * @brief  定义类型, 使用HAL库读写磁编数据时的收发缓冲区
  */
typedef struct
{
    uint16_t            send_data[14U]; // 需要发送的数据缓存, 根据磁编要求最长可以连续写13个数据, 需要外加一个控制字
    uint16_t            recv_data[14U]; // 需要接收的数据缓存, 根据磁编要求最长可以连续读13个数据, 需要外加一个安全字
    uint16_t            send_len;       // 需要发送的数据长度
    uint16_t            recv_len;       // 需要接收的数据长度
} TLE5012_DATA_t;

/**
  * @brief  定义类型, 磁编读写组件
  * @brief  定义类型, 磁编读写组件
  * @brief  定义类型, 磁编读写组件
  */
typedef struct
{
    SPI_TypeDef     *SPIx;          /*!< 用于磁编码器传感器管理的LL库SPI外设 */ /* 用于LL库调用 */
    GPIO_TypeDef    *SPI_CS_Port;   /*!< 磁编使能管脚 */ /* 用于LL库调用 */
    uint16_t        SPI_CS_Pin;     /*!< 磁编使能管脚 */ /* 用于LL库调用 */
    int32_t         speed_factor;   /*!< 存放磁编与速度转换系数 */ // (( digits / 磁编精度 ) / ( 2 * Tupd ))
    TLE5012_DIR_t   dir;            // 确认正方向状态
    TLE5012_REG_t   reg;            // 记录传感器芯片内的各值
    uint16_t        error_code;     // 记录错误状态
    uint16_t        zero;           // 绝对零角度, 15:4bit补偿精度12bit, 3bit计算补偿值时的方向, 0bit代表补偿值是否有效
    bool            set_zero_ok;    // 启动时设置绝对零角度完成
    uint16_t        ( *SSC_send_and_recv )( uint16_t *, uint16_t *, uint16_t, uint16_t );   //指针函数，目的是调用读取SPI数据 /* 用于HAL库调用 */
} TLE5012_Handle_t ;

extern TLE5012_Handle_t  Abs_TLE5012_M1;

/**
  * @brief  使用LL驱动读写数据的API
  * @brief  使用LL驱动读写数据的API
  * @brief  使用LL驱动读写数据的API
  */
    // static __inline bool SSC_LL_send( TLE5012_Handle_t * p_tle_dev, uint16_t tx_data );
    // static __inline bool SSC_LL_recv( TLE5012_Handle_t * p_tle_dev, uint16_t * rx_data, uint8_t size );
void TLE5012_read_value( TLE5012_Handle_t * p_tle_dev, uint16_t reg_value, uint16_t * value, uint8_t size);
void TLE5012_read_angle( TLE5012_Handle_t * p_tle_dev, uint16_t * angle_valuel );
void TLE5012_read_speed( TLE5012_Handle_t * p_tle_dev, uint16_t * angle_speed );
void TLE5012_read_angle_and_speed( TLE5012_Handle_t * p_tle_dev, uint16_t * angle_valuel, uint16_t * angle_speed );


/**
  * @brief  使用HAL驱动读写数据的API, 带CRC校验及状态错误判断
  * @brief  使用HAL驱动读写数据的API, 带CRC校验及状态错误判断
  * @brief  使用HAL驱动读写数据的API, 带CRC校验及状态错误判断
  */
/* CRC 校验函数 */
/* 用于CRC计算的表值, 在 .c文件中实现 */
    // static const uint8_t TableCRC[256];
/* CRC 算法一, 通过查表直接获取, 在 .c文件中实现 */
    // static __inline uint8_t CRC8_TABLE(TLE5012_DATA_t *p_buffer, const uint8_t *TableCRC);
/* CRC 算法二, 通过两个循环硬算, 在 .c文件中实现 */
    // static __inline uint8_t CRC8(TLE5012_DATA_t *p_buffer);
/* 读写函数的SSC通信驱动 */
    // static __inline TLE5012_ERROR_t SSC_send_and_recv(TLE5012_DATA_t *p_buffer);

/* 基础读写函数 */
TLE5012_ERROR_t TLE5012_read_reg( TLE5012_Handle_t * p_tle_dev, TLE5012_CMD_READ_t target_reg, TLE5012_DATA_t * p_buffer, uint16_t length );
TLE5012_ERROR_t TLE5012_write_reg( TLE5012_Handle_t * p_tle_dev, TLE5012_CMD_WRITE_t target_reg, TLE5012_DATA_t * p_buffer, uint16_t length );

/* 定义TLE5012初始化函数 */
void TLE5012_Init( TLE5012_Handle_t * p_tle_dev );

/* 定义API接口 */
TLE5012_ERROR_t TLE5012_ic_reg_backups( TLE5012_Handle_t * p_tle_dev );     // 备份磁编内寄存器值
TLE5012_ERROR_t TLE5012_set_dir( TLE5012_Handle_t * p_tle_dev );            // 设置磁编方向
TLE5012_ERROR_t TLE5012_set_zero_position( TLE5012_Handle_t * p_tle_dev );  // 设置编码器的绝对零角度
TLE5012_ERROR_t TLE5012_git_zero_position( TLE5012_Handle_t * p_tle_dev );  // 获取编码器的绝对零角偏移量
// TLE5012_ERROR_t TLE5012_reset(TLE5012_t *p_tle_dev);                     // 初始化磁编

/* 定义其他功能 */
void New_SPI2_Init(void);
uint16_t SPI2_B11_send_and_recv( uint16_t * p_send_data, uint16_t * p_recv_data, uint16_t send_len, uint16_t recv_len );


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _TLE5012B_H_ */
