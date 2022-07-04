/*
 * Copyright (c)  2022, IoTSharp Development Team
 *
 * SPDX-License-Identifier: LGPLv2.1
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-15     maikebing       the first version
 */
#include <string.h>
#define DBG_ENABLE
#define DBG_SECTION_NAME "adt74xx"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>
#include "adt74xx.h"
#define CRC_POLY 0x31

adt74xx_device_t adt74xx_init(const char *i2c_bus_name, rt_uint8_t adt74xx_addr)
{
    adt74xx_device_t dev;
    RT_ASSERT(i2c_bus_name);

    dev = rt_calloc(1, sizeof(struct adt74xx_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for adt74xx device on '%s' ", i2c_bus_name);
        return RT_NULL;
    }

    if (adt74xx_addr == adt74xx_I2CADDR_DEFAULT)
    {
        dev->adt74xx_addr = adt74xx_addr;
    }
    else
    {
        LOG_E("Illegal adt74xx address:'%x'", adt74xx_addr);
        rt_free(dev);
        return RT_NULL;
    }

    dev->i2c = rt_i2c_bus_device_find(i2c_bus_name);

    if (dev->i2c == RT_NULL)
    {
        LOG_E("Can't find adt74xx_addr device on '%s' ", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    dev->lock = rt_mutex_create("mutex_adt74xx_addr", RT_IPC_FLAG_FIFO);
    if (dev->lock == RT_NULL)
    {
        LOG_E("Can't create mutex for adt74xx_addr device on '%s' ", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    // clear the status register
    atd7422_clear_status(dev);
    // read config
    adt74xx_read_config(dev);
    return dev;
}

void atd7422_deinit(adt74xx_device_t dev)
{
    RT_ASSERT(dev);

    rt_mutex_delete(dev->lock);

    rt_free(dev);
}

// 鍐欏崟涓瘎瀛樺櫒
//reg: 瀵勫瓨鍣ㄥ湴鍧�
//data: 鏁版嵁
// 杩斿洖鍊�: 0, 姝ｅ父 / -1, 閿欒浠ｇ爜
static rt_err_t adt74xx_write_reg(adt74xx_device_t dev, rt_uint8_t reg, rt_uint8_t data)
{
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = { reg, data };

    msgs.addr = dev->adt74xx_addr; /* 浠庢満鍦板潃 */
    msgs.flags = RT_I2C_WR; /* 鍐欐爣蹇� */
    msgs.buf = buf; /* 鍙戦�佹暟鎹寚閽� */
    msgs.len = 2;

    if (rt_i2c_transfer(dev->i2c, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

// 璇诲彇瀵勫瓨鍣ㄦ暟鎹�
//reg: 瑕佽鍙栫殑瀵勫瓨鍣ㄥ湴鍧�
//len: 瑕佽鍙栫殑鏁版嵁瀛楄妭鏁�
//buf: 璇诲彇鍒扮殑鏁版嵁瀛樺偍鍖�
// 杩斿洖鍊�: 0, 姝ｅ父 / -1, 閿欒浠ｇ爜
static rt_err_t adt74xx_read_reg(adt74xx_device_t dev, rt_uint8_t reg, rt_uint8_t *buf, rt_uint8_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = dev->adt74xx_addr; /* 浠庢満鍦板潃 */
    msgs[0].flags = RT_I2C_WR; /* 鍐欐爣蹇� */
    msgs[0].buf = &reg; /* 浠庢満瀵勫瓨鍣ㄥ湴鍧� */
    msgs[0].len = 1; /* 鍙戦�佹暟鎹瓧鑺傛暟 */

    msgs[1].addr = dev->adt74xx_addr; /* 浠庢満鍦板潃 */
    msgs[1].flags = RT_I2C_RD; /* 璇绘爣蹇� */
    msgs[1].buf = buf; /* 璇诲彇鏁版嵁鎸囬拡 */
    msgs[1].len = len; /* 璇诲彇鏁版嵁瀛楄妭鏁� */

    if (rt_i2c_transfer(dev->i2c, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

rt_err_t atd7422_clear_status(adt74xx_device_t dev)
{
    dev->status.word = 0x00;
    //dev->status.status_bits.RDY=1;
    return adt74xx_write_reg(dev, adt74xx_REG_STATUS, dev->status.word);
}
rt_err_t adt74xx_softreset(adt74xx_device_t dev)
{
    rt_err_t ret = adt74xx_write_reg(dev, adt74xx_REG_SWRST, 0x00);
    rt_thread_mdelay(10);
    return ret;
}

rt_err_t adt74xx_check(adt74xx_device_t dev)
{
    rt_uint8_t buf[2] = { 0 };
    rt_err_t ret = RT_EOK;

    if (adt74xx_read_reg(dev, adt74xx_REG_ID, (rt_uint8_t*) &buf, 1) == RT_EOK)
    {
        dev->id.word = buf[0];
        //dev->id.id_word==0xc8;
        if ((buf[0] & 0xF8) == 0xC8) //   dev->id=
        {
            return RT_EOK;
        }
    }
    return ret;
}
rt_err_t adt74xx_read_status(adt74xx_device_t dev)
{
    rt_uint8_t buf[2] = { 0 };
    rt_err_t ret = RT_ERROR;
    if (adt74xx_read_reg(dev, adt74xx_REG_STATUS, (rt_uint8_t*) &buf, 1) == RT_EOK)
    {
        dev->status.word = buf[0];
        ret = RT_EOK;
    }
    return ret;
}

rt_err_t adt74xx_read_config(adt74xx_device_t dev)
{
    rt_uint8_t buf[2] = { 0 };
    rt_err_t ret = RT_EOK;

    if (adt74xx_read_reg(dev, adt74xx_REG_STATUS, (rt_uint8_t*) &buf, 1) == RT_EOK)
    {

        dev->config.word = buf[0];

    }
    return ret;
}
static uint16_t swapShort16(uint16_t shortValue)
{
    return ((shortValue & 0x00FF) << 8) | ((shortValue & 0xFF00) >> 8);
}
rt_err_t adt74xx_read_temperature(adt74xx_device_t dev)
{
    rt_err_t ret = RT_ERROR;
    RT_ASSERT(dev);
    uint16_t temp = 0;
    uint16_t temp1 = 0;
       float temp_c = 0;
    if (adt74xx_read_reg(dev, adt74xx_REG_TEMPMSB, (rt_uint8_t*) &temp1, 2) == RT_EOK)
    {
        temp=swapShort16(temp1);
       //rt_kprintf("adt74xx_read_temperature:%d res=%d\n",temp,dev->config.config_bits.res);
        if(dev->config.config_bits.res) {
                if(temp & 0x8000)
                    /*! Negative temperature */
                    temp_c = (float)((int32_t)temp - 65536) / 128;
                else
                    /*! Positive temperature */
                    temp_c = (float)temp / 128;
            } else {
                temp >>= 3;
                if(temp & 0x1000)
                    /*! Negative temperature */
                    temp_c = (float)((int32_t)temp - 8192) / 16;
                else
                    /*! Positive temperature */
                    temp_c = (float)temp / 16;
            }
        dev->temperature = temp_c;
        ret = RT_EOK;
    }
    return ret;
}

/**
 * This function is exported to MSH commands list
 * Usage example:
 *  - sht3x probe i2c1 pu : initialize sht3x device on i2c1 bus with address pin pulled up(i2c address 0x45)
 *  - sht3x probe i2c1: initialize sht3x device one i2c1 bus with address pin pulled down by default(i2c address 0x44)
 *  - sht3x read: read and print temperature and humidity from previously initialized sht3x
 */
void adt74xx(int argc, char *argv[])
{
    static adt74xx_device_t dev = RT_NULL;
    rt_uint8_t adt_addr = adt74xx_I2CADDR_DEFAULT;

    if (argc > 1)
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (argc >= 3)
            {
                /* initialize the sensor when first probe */
                if (!dev || strcmp(dev->i2c->parent.parent.name, argv[2]))
                {
                    /* deinit the old device */
                    if (dev)
                    {
                        rt_kprintf("Deinit adt74xx");
                        atd7422_deinit(dev);
                    }
                    // no else needed here
                    if (argc > 3)
                    {

                        if (!strcmp("pu", argv[3]))
                        {
                            adt_addr = adt74xx_I2CADDR_DEFAULT;
                        }
                        else
                        {
                            rt_kprintf("Illegal adt74xx address, using 0x44 by default\n");
                            adt_addr = adt74xx_I2CADDR_DEFAULT; // pulled down by default: 0x48
                        }
                    }
                    // no else needed here

                    dev = adt74xx_init(argv[2], adt_addr);
                    if (!dev)
                    {
                        rt_kprintf("adt74xx probe failed, check input args\n");
                    }
                    else
                    {
                        rt_kprintf("adt74xx probed, addr:0x%x\n", adt_addr);
                    }
                }
            }
            else
            {
                rt_kprintf("adt74xx probe <i2c dev name>   - probe sensor by given name\n");
            }
        }
        else if (!strcmp(argv[1], "read"))
        {
            if (dev)
            {
                /* read the sensor data */
                if (adt74xx_read_temperature(dev) == RT_EOK)
                {
                    rt_kprintf("adt74xx temperature: %d.%d \n", (int) dev->temperature,
                            (int) (dev->temperature * 100) % 100);
                }
                else
                {
                    rt_kprintf("Can't read adt74xx temperature: ");
                }

            }
            else
            {
                rt_kprintf("Please using 'adt74xx probe <i2c dev name> <pu/pd>' first\n");
            }
        }
        else if (!strcmp(argv[1], "status"))
        {
            if (dev)
            {
                if (adt74xx_read_status(dev) == RT_EOK)
                {
                    rt_kprintf("adt74xx status:\n");
                    rt_kprintf("\tT_low:\t%d\t- \n", dev->status.status_bits.T_low);
                    rt_kprintf("\tT_high:\t%d\t-\n", dev->status.status_bits.T_high);
                    rt_kprintf("\tT_crit:\t%d\n", dev->status.status_bits.T_crit);
                    rt_kprintf("\tRDY:\t%d\n", dev->status.status_bits.RDY);

                }
                else
                {
                    rt_kprintf("adt74xx status not read\n");
                }
            }
            else
            {
                rt_kprintf("Please using 'adt74xx probe <i2c dev name> <pu/pd>' first\n");
            }
        }
        else if (!strcmp(argv[1], "reset"))
        {
            if (dev)
            {
                if (adt74xx_softreset(dev) == RT_EOK)
                {
                    rt_kprintf("adt74xx reset cmd sent\n");
                }
                else
                {
                    rt_kprintf("adt74xx reset cmd not sent\n");
                }
            }
            else
            {
                rt_kprintf("Please using 'adt74xx probe <i2c dev name> <pu/pd>' first\n");
            }
        }

        else
        {
            rt_kprintf("Unknown command. Enter 'adt74xx' for help\n");
        }
    }
    else
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("\tadt74xx probe <i2c dev name> <pu/pd> -- probe sensor by i2c dev name and pull config\n");
        rt_kprintf("\tadt74xx read -- read sensor adt74xx data\n");
        rt_kprintf("\tadt74xx status -- status register of adt74xx\n");
        rt_kprintf("\tadt74xx reset -- send soft reset command to adt74xx\n");

    }
}

MSH_CMD_EXPORT(adt74xx, adt74xx sensor);

