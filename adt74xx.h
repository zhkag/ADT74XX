/*
 * Copyright (c)  2022, IoTSharp Development Team
 *
 * SPDX-License-Identifier:  LGPLv2.1
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-15     maikebing       the first version
 */
#ifndef APPLICATIONS_adt74xx_H_
#define APPLICATIONS_adt74xx_H_

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#define adt74xx_I2CADDR_DEFAULT 0x48 ///< I2C address

#define adt74xx_REG_TEMPMSB 0x00 ///< Temp. value MSB
#define adt74xx_REG_TEMPLSB 0x01 ///< Temp. value LSB
#define adt74xx_REG_STATUS 0x02  ///< Status register
#define adt74xx_REG_CONFIG 0x03  ///< Configuration register
#define adt74xx_REG_TEMPMSB_HIGH 0x04 ///< Temp. value MSB_HIGH
#define adt74xx_REG_TEMPLSB_HIGH 0x05 ///< Temp. value LSB_HIGH
#define adt74xx_REG_TEMPMSB_LOW 0x06 ///< Temp. value MSB_LOW
#define adt74xx_REG_TEMPLSB_LOW 0x07 ///< Temp. value LSB_LOW
#define adt74xx_REG_TEMPMSB_CRIT 0x08 ///< Temp. value MSB_CRIT
#define adt74xx_REG_TEMPLSB_CRIT 0x09 ///< Temp. value LSB_CRIT
#define adt74xx_REG_THYST 0x0A      ///< Temp. HYST
#define adt74xx_REG_ID 0x0B      ///< Manufacturer identification
#define adt74xx_REG_RSVD1 0x0C      ///< Reserved
#define adt74xx_REG_RSVD2 0x0D      ///< Reserved
#define adt74xx_REG_RSVD3 0x2E      ///< Reserved
#define adt74xx_REG_SWRST 0x2F  ///< Software reset

typedef union _adt74xx_word
{
    rt_uint8_t word;
    struct _id_bits
    {
        rt_uint16_t version : 3 ;
        rt_uint16_t mid : 5 ;
    } id_bits;
    struct _status_bits
      {
          rt_uint16_t unknow : 3 ;
          //温度降至TLOW温度限值以下时，此位置1。读取状态寄存器时和/或所测得温度返回至高于设定点
         // TLOW + THYST寄存器中设置的限值时，该位清0。
          rt_uint16_t T_low : 1 ;
          //温度升至THIGH温度限值以上时，此位置1。读取状态寄存器时和/或所测得温度返回至低于设定点
          //THIGH  HYST寄存器中设置的限值时，该位清0。
          rt_uint16_t T_high : 1 ;
          //温度升至TCRIT温度限值以上时，此位置1。读取状态寄存器时和/或所测得温度返回至低于设定点
          //THYST寄存器中设置的限值时，此位清0。
          rt_uint16_t T_crit : 1 ;
          rt_uint16_t RDY : 1 ;//温度转换结果写入温度值寄存器中时，此位变为低。读取温度值寄存器时，此位复位至1。在单
        //  次采样模式和1 SPS模式下，写入配置寄存器的工作模式位之后，此位复位
      }status_bits;
    struct _config_bits
      {
          rt_uint16_t status : 2 ;//这两个位选择在设置INT和CT引脚之前会发生的欠温/过温故障的数目。这有助于避免          温度噪声所引起的误触发。
          rt_uint16_t CT : 1 ;//此位选择CT引脚的输出极性
          rt_uint16_t INT : 1 ;//此位选择INT引脚的输出极性
          rt_uint16_t INT_CT : 1 ;//此位在比较器模式与中断模式之间进行选择
       //   这两个位设置adt74xx的工作模式。
         // 00 = 连续转换（默认）。一次转换结束后，adt74xx开始另一次转换。
          //01 = 单次采样。转换时间的典型值为240 ms。
          //10 = 1 SPS模式。转换时间的典型值为60 ms。此工作模式降低平均功耗。
          //11 = 关断。关断除接口电路以外的所有电路。
rt_uint16_t mode :2;
          //分辨率0 = 13位分辨率。符号位 + 12位提供温度分辨率0.0625°C。
          //1 = 16位分辨率。符号位 + 15位提供温度分辨率0.0078℃。
          rt_uint16_t res : 1 ;
      } config_bits;
}adt74xx_word;


typedef union _adt74xx_temperature
{
    rt_uint16_t rawvalue;
    struct vlauex
    {
        rt_uint16_t version : 3 ;
        rt_uint16_t mid : 5 ;
    } vlauex;

}_adt74xx_temperature;

struct adt74xx_device
{
    struct rt_i2c_bus_device *i2c;
    rt_uint8_t adt74xx_addr ;
    rt_mutex_t lock;
    float temperature ;
    adt74xx_word id;//厂商信息
    adt74xx_word config;//配置
    adt74xx_word status;//状态
};
typedef struct adt74xx_device *adt74xx_device_t;
adt74xx_device_t adt74xx_init(const char *i2c_bus_name, rt_uint8_t adt74xx_addr);
rt_err_t adt74xx_read_status(adt74xx_device_t dev);
rt_err_t adt74xx_read_temperature(adt74xx_device_t dev);
void adt74xx_deinit(adt74xx_device_t dev);
rt_err_t adt74xx_clear_status(adt74xx_device_t dev);
rt_err_t adt74xx_read_config(adt74xx_device_t dev);
#endif /* APPLICATIONS_adt74xx_H_ */
