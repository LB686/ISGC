# INA303 电流/电压采集驱动 集成说明

## 一、文件清单

| 文件 | 路径 | 说明 |
|------|------|------|
| 配置文件 | `BSP/inc/bsp_ina303_cfg.h` | 所有硬件参数宏定义，移植时只需改此文件 |
| 驱动头文件 | `BSP/inc/bsp_ina303.h` | API声明、枚举、状态定义 |
| 驱动实现 | `BSP/src/bsp_ina303.c` | DMA双缓冲、滤波、计算、保护逻辑 |

---

## 二、CUBEMX 配置步骤（关键！）

### 1. ADC 配置
1. 打开 STM32CubeMX，进入 **ADC1**（或你实际使用的ADC）配置
2. 使能以下通道（根据你实际原理图选择对应引脚）：
   - `IN0` ~ `INx`：3路INA303输出（U/V/W相电流）
   - `INy`：母线电压分压后输入
3. 配置参数：
   - **Resolution**: 12 bits (15 ADC Clock cycles)
   - **Scan Conversion Mode**: Enabled（扫描模式）
   - **Continuous Conversion Mode**: Enabled（连续转换）
   - **DMA Continuous Requests**: Enabled
   - **End Of Conversion Selection**: EOC flag at the end of all conversions
4. 在 **Rank** 配置中，确认扫描顺序（ Rank 1 ~ 4 分别对应哪个通道）
5. **Sampling Time**: 建议设为 **15 Cycles** 或更长，确保输入阻抗匹配

### 2. DMA 配置
1. 为ADC1添加DMA请求
2. **Direction**: Peripheral To Memory
3. **Mode**: **Circular**（循环模式，必须！）
4. **Data Width**: Peripheral=Half Word, Memory=Half Word
5. **Increment Address**: Memory勾选，Peripheral不勾选

### 3. NVIC 中断配置
1. 开启 **ADC1 DMA IRQ**（优先级根据系统需求设置，建议低于PWM故障中断）
2. 如需使用INA303硬件Alert引脚，开启对应 **EXTI Line** 中断

---

## 三、修改配置文件

打开 `BSP/inc/bsp_ina303_cfg.h`，根据实际硬件修改以下宏：

```c
/* 1. ADC句柄 */
#define INA303_ADC_HANDLE               hadc1   /* 或 hadc2 / hadc3 */

/* 2. ADC扫描顺序（必须与CUBEMX中Rank 1~4的顺序完全一致） */
#define INA303_SCAN_IDX_IU              0       /* Rank 1 = U相 */
#define INA303_SCAN_IDX_IV              1       /* Rank 2 = V相 */
#define INA303_SCAN_IDX_IW              2       /* Rank 3 = W相 */
#define INA303_SCAN_IDX_VBUS            3       /* Rank 4 = 母线电压 */

/* 3. 分压电阻（根据实际原理图填写！） */
#define INA303_BUS_DIV_R1_OHM           18000.0f    /* 上电阻 */
#define INA303_BUS_DIV_R2_OHM           1000.0f     /* 下电阻 */

/* 4. 保护阈值（根据电机和功率管规格修改） */
#define INA303_OC_THRESHOLD_A           65.0f
#define INA303_UC_THRESHOLD_A           (-65.0f)
#define INA303_OV_THRESHOLD_V           50.0f
#define INA303_UV_THRESHOLD_V           15.0f
```

**特别注意分压比**：
- 默认 `18k:1k` 分压比约 `1/19`，最大量程约 **62.7V**
- 如果你的发电阶段反电动势可能超过60V，请减小分压比（如改为 `30k:1k`，量程约102V）

---

## 四、代码集成

### 1. 在 `bsp.h` 中包含驱动头文件

```c
#include "../../bsp/inc/bsp_ina303.h"
```

### 2. 在 `main.c` 或 `bsp.c` 的初始化中启动驱动

```c
void bsp_Init(void)
{
    /* ... 其他初始化 ... */
    BSP_INA303_Init();      /* 启动INA303 ADC DMA采集 */
}
```

### 3. 在 DMA 中断回调中插入驱动处理

找到 `Core/Src/stm32f4xx_it.c` 或 CUBEMX生成的 `main.c` 中的以下函数：

```c
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &INA303_ADC_HANDLE) {
        BSP_INA303_DMAHalfCplt();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &INA303_ADC_HANDLE) {
        BSP_INA303_DMACplt();
    }
}
```

> 如果CUBEMX把这两个回调定义在 `main.c` 中，直接在里面添加即可。

### 4. 在快速循环中调用处理函数

**方案A：如果你已有1kHz以上的定时中断（如PWM中断）**

```c
void TIMx_UP_IRQHandler(void)   /* 或你的PWM更新中断 */
{
    /* ... */
    BSP_INA303_FastTask();      /* 执行滤波、计算、保护检测 */
    /* ... */
}
```

**方案B：使用已有的 hal_fast_loop()（如果有高频调用）**

```c
void hal_fast_loop(void)
{
    BSP_INA303_FastTask();
}
```

**方案C：如果没有高频中断，可在 `hal_1000hz_loop()` 中调用**

```c
void hal_1000hz_loop(void)
{
    BSP_INA303_FastTask();
}
```

> **注意**：`FastTask` 调用频率直接影响保护响应速度和滤波效果。电机控制建议放在 **PWM频率中断**（如10kHz）中。

---

## 五、使用API获取数据

```c
float iu, iv, iw, vbus;

iu   = BSP_INA303_GetCurrentU();      /* U相电流，单位A，发电时为负 */
iv   = BSP_INA303_GetCurrentV();      /* V相电流 */
iw   = BSP_INA303_GetCurrentW();      /* W相电流 */
vbus = BSP_INA303_GetBusVoltage();    /* 母线电压，单位V */

/* 故障检测 */
if (BSP_INA303_IsOverCurrent()) {
    /* 过流保护动作 */
}

if (BSP_INA303_IsUnderCurrent()) {
    /* 欠流保护动作（发电负向过流） */
}

if (BSP_INA303_IsOverVoltage()) {
    /* 过压保护动作（发电阶段反电动势过高） */
}
```

---

## 六、零偏校准（上电后必须执行！）

INA303和采样电阻存在个体差异，**必须在0A条件下进行零偏校准**：

```c
/* 在系统初始化完成、电机尚未启动时调用 */
void SystemInit_ZeroCal(void)
{
    BSP_INA303_CalibrateZeroStart();    /* 启动校准 */
    
    /* 等待校准完成（约64ms @ 1kHz FastTask） */
    while (BSP_INA303_CalibrateZeroPoll() != INA303_CAL_DONE) {
        HAL_Delay(1);
    }
    
    /* 校准完成，后续读取的电流即为准确值 */
}
```

> **警告**：校准时必须确保三相电流真实为0A（电机停转、功率管关闭），否则校准结果错误！

---

## 七、可选：增益校准（高精度场合）

如果实测电流与标准电流源存在系统误差（如采样电阻精度、INA303增益误差），可修正增益系数：

```c
/* 通入已知电流（如50A），读取驱动输出值 */
float readCurrent = BSP_INA303_GetCurrentU();   /* 假设读数为48.5A */
float trueCurrent = 50.0f;
float gainCal = trueCurrent / readCurrent;      /* 50 / 48.5 ≈ 1.031 */

BSP_INA303_SetGainCal(INA303_CH_IU, gainCal);
BSP_INA303_SetGainCal(INA303_CH_IV, gainCal);
BSP_INA303_SetGainCal(INA303_CH_IW, gainCal);
```

---

## 八、关键参数速查

| 参数 | 值 | 说明 |
|------|-----|------|
| 采样电阻 | 0.2 mΩ | |
| INA303A1增益 | 20 V/V | |
| 电流分辨率 | ~0.20 A/LSB | 3.3V/4096 / (0.2mΩ×20) |
| 0A时ADC理论值 | 2048 | REF=1.65V |
| +70A时ADC值 | ~2188 | |
| -70A时ADC值 | ~1908 | |
| 母线分压比(默认) | 1/19 | R1=18k, R2=1k |
| 电压分辨率 | ~0.0089 V/LSB | |
| 滤波深度 | 8点平均 | 可配置为4/16/32点 |
| 保护消抖 | 5ms @ 1kHz | 可配置 |

---

## 九、常见问题排查

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| 电流始终为正值 | REF引脚接地（单向模式） | 检查INA303 REF引脚是否接1.65V |
| 0A时电流不为0 | 未做零偏校准 | 上电后执行 `BSP_INA303_CalibrateZeroStart()` |
| 电流数值跳动大 | 滤波深度不够或ADC采样时间短 | 增大`INA303_FILTER_SHIFT`，增加CUBEMX中Sampling Time |
| 发电时电压饱和 | 分压比太大 | 减小分压比，确保 `Vbus_max × R2/(R1+R2) < 3.3V` |
| 过流误触发 | 阈值设太低或消抖不够 | 调整`INA303_OC_THRESHOLD_A`和`INA303_PROT_DEBOUNCE_CNT` |
| DMA不进入中断 | CUBEMX未开启DMA Circular模式 | 检查DMA Mode是否为Circular |

---

## 十、后续扩展建议

1. **接入硬件Alert引脚**：将INA303的Alert1/Alert2接到MCU外部中断，在`EXTIx_IRQHandler`中调用 `BSP_INA303_Alert1_IRQHandler()`，实现 <1us 的硬件过流保护
2. **温度补偿**：如采样电阻温漂大，可外接NTC温度传感器，根据温度实时修正`gainCal`
3. **三相电流和校验**：电机控制中可利用 `iu + iv + iw = 0` 进行冗余校验，发现传感器故障
