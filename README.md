# 自动行驶小车（2024 电赛 H 题）

基于 **TI MSPM0G3507** 的轮式自动行驶小车工程。小车在 220×120 cm 白底场地上沿两段黑色半圆弧（半径 40 cm）行驶，依次经过 A/B/C/D 顶点，完成题目要求 (1)~(4)，每过顶点和停车时以 LED + 串口声光提示。

- 感知：4 路红外巡线 + 双轮增量编码器
- 控制：双环串级（外环 巡线PD / 航向PD → 左右目标转速；内环 每轮 PI 速度环 → PWM）
- 调度：事件驱动状态机（直线 ⟶ 抓到黑线 ⟶ 巡线 ⟶ 全白过顶 ⟶ 直线），不写死里程
- 调参：UART0 在线改 PID / 基础速度，免重编译

> 工程主体在 `2024H_project/`。`直线行驶参考/` 为速度环参考工程，`硬件连接线路/` 为接线说明，`H题_自动行驶小车.pdf` 为原题。

---

## 一、目录结构

```
new_2024H/
├─ 2024H_project/              # 主工程
│  ├─ empty.c                  # main：初始化、按键选题、主循环
│  ├─ empty.syscfg            # SysConfig 引脚/外设配置源
│  ├─ ti_msp_dl_config.c/.h   # SysConfig 生成（构建时自动重生成）
│  ├─ Hardware/               # 驱动层
│  │  ├─ motor.c/.h           #   PWM 输出（仅前进）
│  │  ├─ encoder.c/.h         #   正交编码计数 + 转速换算
│  │  ├─ ir.c/.h              #   4 路红外读取、加权偏差、丢线判定
│  │  ├─ led.c/.h key.c/.h    #   LED、按键
│  │  └─ board.c/.h           #   SysTick 延时、printf→UART0
│  ├─ Control/                # 控制层
│  │  ├─ control.c/.h         #   10ms tick 双环串级控制
│  │  ├─ path.c/.h            #   事件驱动路径状态机
│  │  └─ tune.c/.h            #   UART 在线调参
│  ├─ keil/                   # Keil µVision 工程 + 输出
│  ├─ source/ ti/ tools/      # MSPM0 SDK 依赖、DriverLib、syscfg 脚本
│  ├─ 设计报告.md / .pdf       # 设计报告
│  └─ README.md
├─ 直线行驶参考/               # 速度环参考工程
├─ 硬件连接线路/               # 接线说明
└─ H题_自动行驶小车.pdf        # 原题
```

---

## 二、硬件信息

| 项目 | 说明 |
| :-- | :-- |
| MCU | TI MSPM0G3507（Cortex-M0+，80 MHz，LQFP-64） |
| 电机驱动 | A4950（双路 H 桥） |
| 电机 | 带 13 线增量编码器 + 30:1 减速箱（每轮 780 计数/转） |
| 传感器 | 4 路红外巡线模块（黑线有效，灯亮） |
| 提示 | LED（PB9）+ UART0 串口（115200） |
| 供电 | 车载电池（题目要求中途不换电池） |

---

## 三、硬件接线

> 引脚以 `2024H_project/empty.syscfg` 与生成的 `ti_msp_dl_config.h` 为准。

| 模块 | 信号 | MCU 引脚 | 类型 |
| :-- | :-- | :-- | :-- |
| 电机 PWM | 左轮 / 右轮 | **PB2 / PB3** | TIMA1 CCP0/CCP1 |
| 电机方向 | AIN1/AIN2/BIN1/BIN2 | PA14 / PA15 / PA16 / PA17 | 输出（固定前进） |
| 左编码器 | A 相 / B 相 | **PA25 / PA26** | 输入中断 |
| 右编码器 | A 相 / B 相 | **PB20 / PB24** | 输入中断 |
| 红外巡线 | DH1 / DH2 / DH3 / DH4 | **PA27 / PA12 / PB16 / PB17** | 输入（上拉） |
| 按键 | KEY | PA18 | 输入 |
| 指示灯 | LED | PB9 | 输出（低电平点亮） |
| 调试串口 | UART0 TX / RX | PA10 / PA11 | 115200 8N1 |
| 下载调试 | SWCLK / SWDIO | PA20 / PA19 | SWD |

**接线要点**
- 所有外设必须与 MCU **共地**。
- 红外四路从左到右依次为 DH1–DH4，正轨行驶时**中间两路（DH2/DH3）压黑线**（状态 `0110`）。
- 编码器 A/B 接反 → 计数方向相反；电机线序接反 → 转向相反。优先改线，再改软件极性。
- 串口接法：MCU TX→外设 RX，MCU RX→外设 TX。

> ⚠️ 注意：`empty.syscfg` 将 **PA15 用作电机 AIN2**，而旧接线表中 PA15 标为电池 ADC，二者不可共用。本工程以 syscfg 为准（PA15=AIN2）。

---

## 四、软件环境

| 工具 | 版本 |
| :-- | :-- |
| Keil µVision | MDK-ARM，编译器 ARMCLANG V6.21 |
| TI SysConfig | 1.20.0 |
| MSPM0 SDK | 2.01.00.03 |

构建时 Keil 的 **BeforeMake** 会调用 `tools/keil/syscfg.bat` 自动从 `empty.syscfg` 重新生成 `ti_msp_dl_config.c/.h`。该脚本内的工具路径需指向本机安装位置：

```bat
set SYSCFG_PATH="E:\ti\sysconfig-1.20.0_3587\sysconfig_cli.bat"
set SDK_ROOT=E:\ti\mspm0_sdk_2_01_00_03
```

若你的安装路径不同，请修改 `2024H_project/tools/keil/syscfg.bat` 顶部这两行。

---

## 五、编译与下载

1. 用 Keil µVision 打开
   `2024H_project/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
2. **Project → Rebuild**（会先跑 SysConfig 重生成配置，再编译）。
   - 期望结果：`0 Error(s), 0 Warning(s)`，生成 `keil/Objects/*.axf` 与 `*.hex`。
3. 通过 SWD（CMSIS-DAP / XDS）下载到目标板，或用 `*.hex` 烧录。

命令行批量编译（可选）：
```bash
"E:\MDK_ARM\Keil_v5\UV4\UV4.exe" -b \
  "2024H_project\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx" -o build_log.txt
```

> 注：SysConfig 每次会重新注入 `DL_UART_Main_enableLoopbackMode`，`main()` 中已在 `SYSCFG_DL_init()` 后调用 `DL_UART_Main_disableLoopbackMode` 兜底，保证真实串口可用。

---

## 六、运行步骤

1. 上电，连接 UART0（115200 8N1），可见启动 banner。
2. **单击按键**循环选择题目要求 REQ 1→2→3→4。
3. **双击按键**启动。小车从直线状态出发：
   - 红外抓到黑弧 → 进入巡线；四路全白（过顶点）→ 计数 +1、LED 闪 + 串口打印 → 回到直线。
   - 达到该要求的顶点数 → 停车，LED 常亮 + 打印 `ARRIVED`。
4. 完成后双击可重新选择并再次运行。

各要求经过的顶点数：REQ1=1（到 B），REQ2/REQ3=4（回到 A），REQ4=16（4 圈×4 顶点）。现场可按实际红外跳变核对。

---

## 七、串口在线调试

运行中通过 UART0 直接发命令改参数，**无需重编译烧录**（行尾用回车/换行）：

| 命令 | 说明 | 示例 |
| :-- | :-- | :-- |
| `p` | 打印全部增益 | `p` |
| `g <id> <val>` | 设置增益 id（0~7） | `g 6 5.0` |
| `b <val>` | 设置基础速度（RPM） | `b 60` |
| `f <val>` | 设置弧线前馈偏置（RPM） | `f 8` |

增益 id：`0`KpSpd `1`KiSpd `2`FFL `3`FFR `4`KpHead `5`KdHead `6`KpLine `7`KdLine

实时遥测（运行中每 ~200 ms 一行）：
```
st=2 v=1 IR=0x6 Lrpm=54.3 Rrpm=55.1
```
`st` 控制模式（0停/1直线/2巡线），`v` 顶点计数，`IR` 红外状态，`Lrpm/Rrpm` 左右轮转速。

---

## 八、现场整定建议（按优先级）

1. **红外极性**：确认黑线下传感器读数。若巡线方向相反，改 `Hardware/ir.h` 的 `IR_BLACK_IS_HIGH`。确认中间两路压线时为 `0110`。
2. **编码器方向**：确认 `Lrpm/Rrpm` 在前进时为正。为负说明 A/B 或线序接反。
3. **基础速度 `b`**：从低速起调，确认能稳定直行与巡线。
4. **巡线 PD `g 6 / g 7`**：先调 `KpLine` 到能贴线，再加 `KdLine` 抑制振荡（注意红外偏差是离散值，KdLine 不宜过大）。
5. **航向 PD `g 4 / g 5`**：直线段不跑偏即可。
6. **弧线前馈 `f`**：给非零偏置分担 40 cm 转弯，减轻 PD 负担、降低脱线风险。
7. **顶点去抖 / 丢线确认**：见 `Control/path.c` 的 `ACQUIRE_N`、`VERTEX_LOST_N`（需改代码重编译）。

---

## 九、已知约束

- 小车**仅前进**，不倒车；每段半圆弧自然反转方向 180°，无需倒车即可闭合路径。
- 本工程已在目标工具链下编译通过并经 SysConfig 端口校验；**PID 参数为整定起点**，需在实物上按上节步骤整定后方能稳定跑完全程。
- 编译/下载需在装有 Keil + SysConfig + MSPM0 SDK 的机器上进行。
