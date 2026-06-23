# 硬件连接线路说明

本文档按当前工程实际配置整理，优先以 `empty.syscfg` 和 `ti_msp_dl_config.h` 的生效内容为准。

说明：
- MCU 为 `MSPM0G3507`
- 所有外设信号默认按 3.3V 逻辑接入
- 所有外设地必须与 MCU 共地
- `Hardware/IR_Module.h` 里有少量旧宏名，不作为最终接线依据

## 一、整体拓扑

- MCU -> 电机驱动 -> 左/右电机
- MCU -> 左/右编码器 -> 速度反馈
- MCU -> 4 路红外巡线传感器 -> 巡线状态
- MCU -> OLED 显示屏 -> 状态显示
- MCU -> MPU6050 (I2C0) -> 航向角(yaw)反馈
- MCU -> 按键 / 指示灯
- MCU -> 电池电压采样
- MCU -> UART0 调试串口
- MCU -> UART1 蓝牙/APP 串口
- MCU -> SWD 调试下载口

## 二、引脚总表

| 模块 | 信号 | MCU 引脚 | 类型 | 软件位置 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 电机 PWM | 左轮 PWM | PB2 | 输出 | `ti_msp_dl_config.h`, `Hardware/motor.c` | `TIMA1 CCP0`，左轮速度控制 |
| 电机 PWM | 右轮 PWM | PB3 | 输出 | `ti_msp_dl_config.h`, `Hardware/motor.c` | `TIMA1 CCP1`，右轮速度控制 |
| 电机驱动 | AIN1 | PA14 | 输出 | `ti_msp_dl_config.h`, `Hardware/motor.c` | 左侧电机方向/驱动控制脚 |
| 电机驱动 | AIN2 | PA13 | 输出 | `ti_msp_dl_config.h` | 左侧电机方向/驱动控制脚 |
| 电机驱动 | BIN1 | PA16 | 输出 | `ti_msp_dl_config.h` | 右侧电机方向/驱动控制脚 |
| 电机驱动 | BIN2 | PA17 | 输出 | `ti_msp_dl_config.h`, `Hardware/motor.c` | 右侧电机方向/驱动控制脚 |
| 编码器 | 左编码器 A 相 | PA26 | 输入中断 | `ti_msp_dl_config.h`, `Hardware/encoder.c` | 左轮脉冲计数 |
| 编码器 | 左编码器 B 相 | PA25 | 输入中断 | `ti_msp_dl_config.h`, `Hardware/encoder.c` | 左轮方向判断 |
| 编码器 | 右编码器 A 相 | PB24 | 输入中断 | `ti_msp_dl_config.h`, `Hardware/encoder.c` | 右轮脉冲计数 |
| 编码器 | 右编码器 B 相 | PB20 | 输入中断 | `ti_msp_dl_config.h`, `Hardware/encoder.c` | 右轮方向判断 |
| 巡线传感器 | DH1 | PA27 | 输入 | `ti_msp_dl_config.h`, `Hardware/IR_Module.c` | 4 路巡线中的一位 |
| 巡线传感器 | DH2 | PA12 | 输入 | `ti_msp_dl_config.h`, `Hardware/IR_Module.c` | 4 路巡线中的一位 |
| 巡线传感器 | DH3 | PB16 | 输入 | `ti_msp_dl_config.h`, `Hardware/IR_Module.c` | 4 路巡线中的一位 |
| 巡线传感器 | DH4 | PB17 | 输入 | `ti_msp_dl_config.h`, `Hardware/IR_Module.c` | 4 路巡线中的一位 |
| OLED | RST | PB14 | 输出 | `ti_msp_dl_config.h`, `Hardware/oled.c` | OLED 复位 |
| OLED | DC | PB15 | 输出 | `ti_msp_dl_config.h`, `Hardware/oled.c` | OLED 数据/命令选择 |
| OLED | SCL | PA28 | 输出 | `ti_msp_dl_config.h`, `Hardware/oled.c` | 软件时钟线 |
| OLED | SDA | PA31 | 输出 | `ti_msp_dl_config.h`, `Hardware/oled.c` | 软件数据线 |
| MPU6050 | SDA | PA0 | I2C | `ti_msp_dl_config.h`, `Hardware/i2c.c` | 硬件 I2C0 数据线，需上拉 |
| MPU6050 | SCL | PA1 | I2C | `ti_msp_dl_config.h`, `Hardware/i2c.c` | 硬件 I2C0 时钟线，需上拉 |
| 按键 | KEY | PA18 | 输入 | `ti_msp_dl_config.h`, `Hardware/key.c` | 按下为低电平 |
| 指示灯 | LED | PB9 | 输出 | `ti_msp_dl_config.h`, `Hardware/led.c` | 低电平点亮 |
| 电压采样 | BAT_ADC | PA15 | 模拟输入 | `ti_msp_dl_config.h`, `Hardware/adc.c` | 电池电压采样，软件按分压比 11 计算 |
| 调试串口 | UART0 TX | PA10 | 输出 | `ti_msp_dl_config.h`, `Hardware/board.c` | `printf` 重定向输出 |
| 调试串口 | UART0 RX | PA11 | 输入 | `ti_msp_dl_config.h` | 调试串口接收 |
| 蓝牙串口 | UART1 TX | PB6 | 输出 | `ti_msp_dl_config.h`, `Control/uart_callback.c` | 蓝牙/APP 通信 |
| 蓝牙串口 | UART1 RX | PB7 | 输入 | `ti_msp_dl_config.h`, `Control/uart_callback.c` | 蓝牙/APP 通信，DMA 接收 |
| 调试下载 | SWCLK | PA20 | 调试 | `ti_msp_dl_config.h` | SWD 时钟 |
| 调试下载 | SWDIO | PA19 | 调试 | `ti_msp_dl_config.h` | SWD 数据 |

## 三、各模块接线说明

### 1. 电机与驱动

电机控制分成两部分：
- PWM 占空比输出：PB2、PB3
- 方向/驱动控制脚：PA14、PA13、PA16、PA17

当前程序里，`Hardware/motor.c` 直接把 PWM 写到 `PB2/PB3`，并保持方向控制脚处于固定巡线配置。也就是说，这套工程目前主要按“前进巡线”来用，未把反向控制做成上层功能。

接线要求：
- 左电机 PWM 接 PB2
- 右电机 PWM 接 PB3
- 电机驱动板的四个方向脚分别接 PA14 / PA13 / PA16 / PA17
- 驱动板地和 MCU 地必须相连

如果出现左右方向相反，优先检查电机线序或方向脚逻辑，再改软件。

### 2. 编码器

左右轮都是 A/B 两相增量编码器。

- 左编码器 A/B：PA26 / PA25
- 右编码器 A/B：PB24 / PB20

`Hardware/encoder.c` 用双相信号判断脉冲方向。接线时要保证：
- A 相、B 相不要接反
- 编码器电源和 MCU 共地
- 线长较长时注意抗干扰

如果出现计数方向与实际运动相反，优先交换同一编码器的 A/B 两线，或者再改软件极性。

### 3. 红外巡线传感器

本工程使用 4 路巡线输入：
- DH1 -> PA27
- DH2 -> PA12
- DH3 -> PB16
- DH4 -> PB17

软件在 `Hardware/IR_Module.c` 里把 4 路状态组合成 4 bit 状态：

```c
sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;
```

当前巡线逻辑按“黑线有效”处理，软件中会把 GPIO 读到的电平做一次翻转。你现场调试时可以按这个原则理解：
- 黑线命中 -> 对应位有效
- 白底 -> 对应位无效

程序里把 `0110` 当作直行状态，也就是中间两路在黑线上。

### 4. OLED 显示屏

OLED 不是硬件 I2C，而是用 GPIO 模拟时序写入：
- RST -> PB14
- DC -> PB15
- SCL -> PA28
- SDA -> PA31

`Hardware/oled.c` 负责 OLED 初始化和刷新。接线时注意：
- 这 4 根线都要接对
- OLED 供电和 MCU 逻辑电平要匹配
- 如果屏幕不亮，先看 RST/DC/SCL/SDA 是否接反

### 5. MPU6050 航向传感器（IMU）

MPU6050 走的是**硬件 I2C0**（不是软件模拟），用于读 Z 轴陀螺仪，积分出航向角（yaw），给直行/盲走过白纸区做航向闭环纠偏：
- SDA -> PA0
- SCL -> PA1
- VCC -> 3.3V，GND -> 共地，AD0 -> 接地（或悬空），器件地址 0x68

`Hardware/i2c.c` 是 I2C0 底层读写，`Hardware/mpu6050.c` 负责初始化、零偏校准和 yaw 积分。接线/使用要点：
- **SDA/SCL 必须各接一个上拉电阻到 3.3V**（4.7kΩ 左右）。很多 MPU6050 模块板载已带上拉，若用裸芯片必须自己加，否则 I2C 读不到（开机串口会打印 `MPU6050 NOT found`）。
- I2C0 速率约 100kHz，逻辑 3.3V。
- 开机后会做**静止零偏校准**（约 0.8s），这期间**车必须静止不动**，否则零偏不准、yaw 会持续漂移。
- 模块要**水平、牢固**固定在车上（Z 轴朝上），松动会引入额外角度误差。
- 若没接 MPU6050 或读不到，程序自动回退到“编码器左右轮里程差”做航向保持，仍可运行，只是过弯出弯后直线精度差一些。

### 6. 按键与指示灯

- 按键 KEY -> PA18，按下为低电平
- LED -> PB9，低电平点亮

也就是说：
- 按键是低有效
- 指示灯是低有效

### 7. 电池电压采样

电池电压采样脚是：
- PA15 -> ADC1 通道

`Hardware/adc.c` 里按分压比换算电压，当前软件使用的换算系数是 11 倍分压。若以后改了分压电阻，需要同步改 `Hardware/adc.c` 的换算公式。

### 8. 串口通信

#### UART0

- PA10 -> TX
- PA11 -> RX

`Hardware/board.c` 把 `printf` 重定向到了 UART0，所以它是调试输出串口。你看串口打印、上位机日志、调试信息，主要走这一路。

#### UART1

- PB6 -> TX
- PB7 -> RX

这一组是蓝牙/APP 通信口，`Control/uart_callback.c` 里通过 DMA 接收。当前工程如果不用蓝牙，可以不接外设，但引脚配置保留着。

串口接线原则：
- MCU TX 接外设 RX
- MCU RX 接外设 TX
- 地线必须共地

### 9. SWD 调试下载

- PA20 -> SWCLK
- PA19 -> SWDIO

这是烧录和在线调试必需的接口。不要把这两个脚再拿去接普通外设。

## 四、软件文件对应关系

| 文件 | 作用 |
| --- | --- |
| `empty.syscfg` | 全局引脚复用、外设实例配置的源头 |
| `ti_msp_dl_config.h/c` | SysConfig 生成的实际引脚和外设初始化 |
| `Hardware/motor.c` | 电机 PWM 输出 |
| `Hardware/encoder.c` | 编码器脉冲计数 |
| `Hardware/IR_Module.c` | 4 路巡线传感器读取和巡线状态机 |
| `Hardware/oled.c` | OLED 驱动 |
| `Hardware/i2c.c` | 硬件 I2C0 底层读写（MPU6050 用） |
| `Hardware/mpu6050.c` | MPU6050 初始化、零偏校准、yaw 角积分 |
| `Hardware/key.c` | 按键扫描 |
| `Hardware/led.c` | LED 控制 |
| `Hardware/adc.c` | 电池电压采样 |
| `Hardware/board.c` | `printf` 重定向到 UART0 |
| `Control/uart_callback.c` | UART1 蓝牙/APP 数据解析 |
| `Control/show.c` | OLED / 串口显示数据组织 |
| `Control/control.c` | 速度闭环和巡线控制调度 |

## 五、现场接线检查顺序

1. 先确认 MCU、传感器、驱动板、电机、编码器全部共地。
2. 先接 PWM 和方向脚，再测电机能否单独转动。
3. 再接编码器，看速度反馈方向和实际是否一致。
4. 再接 4 路红外，看中间两路是否能稳定识别直行。
5. 再接 OLED，确认显示数据正常。
6. 接 MPU6050（PA0/PA1，带上拉），上电看串口是否打印 `MPU6050 OK`；校准时车保持静止。
7. 最后接 UART0 / UART1 做打印和 APP 联调。

## 六、补充说明

- 当前工程以巡线为主，正常工作时不依赖蓝牙也能跑。
- 如果代码和某个旧头文件里的宏名不一致，优先看 `empty.syscfg` 和 `ti_msp_dl_config.h`。
- 若要改接线，优先改 SysConfig，再重新生成配置文件，避免手工改动后被覆盖。
