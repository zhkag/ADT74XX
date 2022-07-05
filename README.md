# ADT74XX

## 1、介绍

这是一个在RT-Thread上，ADT74XX系列温度传感器的驱动。 实现了高精度温度的读取，并支持`Finsh/MSH`测试命令。
本驱动编程实现方便读取多个ADT74XX传感器。地址IO的控制和数据的滤波留给用户自行编程处理。

### 1.1 目录结构

| 名称 | 说明 |
| ---- | ---- |
| adt74xx.h  | 头文件 |
| adt74xx.c  | 源代码 |
| SConscript | RT-Thread 默认的构建脚本 |
| README.md | 软件包使用说明 |

### 1.2 许可证

ADT74XX package 遵循 `LGPLv2.1` 许可，详见 `LICENSE` 文件。

### 1.3 依赖

依赖 `RT-Thread I2C` 设备驱动框架。

## 2、如何获取软件包

使用 sht3x package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    peripheral libraries and drivers  --->
        [*] adt74xx: digital temperature sensor adt7422  driver library
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 3、使用 adt74xx

本驱动实现基于RT-Thread的I2C设备驱动框架，在打开 adt74xx package 后，请务必打开至少1个I2C设备。
 

## 4、注意事项

- 请注意adt74xx芯片的地址引脚默认地址 0x48

## 5、联系方式 & 感谢

* 维护：麦壳饼
* 邮箱：mysticboy@live.com
* 主页：https://github.com/iotsharp/adt74xx
