# 大疆C板的IMU卡尔曼解算使用说明

##### 作者：王家豪

##### 最终更新时间 2026.02.21

## 基本说明

本算法基于扩展卡尔曼滤波（EKF），以四元数作为核心姿态表示，对 IMU 输出的六轴原始数据（陀螺仪与加速度计）进行融合处理。

**状态向量（6维）：** `[q0, q1, q2, q3, bias_x, bias_y]`（四元数 + 陀螺仪 xy 轴零偏）

**观测向量（3维）：** 归一化加速度计三轴数据

同时提供可选的卡方检验机制：当检测到当前周期内的加速度超出可信范围时，将自动屏蔽加速度量测输入，从而避免高动态环境下因线加速度过大而导致的观测误差，确保姿态解算过程以陀螺仪预测为主，保持计算的稳定性和精度。

## 代码测试环境

大疆 C 型开发板，BMI088，基于 HAL 库的 GMaster 代码库 GSRL

## 代码位置

- `GSRL/Algorithm/inc/alg_ahrs.hpp` — AHRS 基类与 QuaternionEKF 子类声明
- `GSRL/Algorithm/src/alg_ahrs.cpp` — AHRS 基类与 QuaternionEKF 子类实现
- `GSRL/Algorithm/inc/alg_filter.hpp` — 底层卡尔曼滤波器模板类

## 类层次结构

```
AHRS (基类)
├── initQuaternion()          — 根据传感器数据初始化四元数(9轴TRIAD / 6轴gravity-only)
├── calculateMotionAccel()    — 计算运动加速度(机体系 + 大地系)
├── convertQuaternionToEulerAngle()
├── update()                  — 统一调用入口
├── Mahony (子类)             — Mahony PI互补滤波
└── QuaternionEKF (子类)      — 四元数扩展卡尔曼滤波
```

## 类接口说明

### 构造函数

```cpp
QuaternionEKF(fp32 sampleFreq         = 0.0f,   // 给定刷新频率(Hz), 给0则使用DWT自动识别
              fp32 quatProcessNoise   = 10.0f,  // 四元数过程噪声基准 (Q矩阵, 典型值10)
              fp32 biasProcessNoise   = 0.001f, // 陀螺仪零偏过程噪声基准 (Q矩阵, 典型值0.001)
              fp32 measNoise          = 1e6f,   // 加速度计量测噪声基准 (R矩阵, 典型值1e6)
              fp32 lambda             = 1.0f,   // 衰减系数(0~1], 防止零偏协方差过度收敛, 1为不衰减
              fp32 accLpfCoef         = 0.0f,   // 加速度计一阶低通滤波时间常数(s), 0表示不滤波
              bool isCheckChiSquare   = true,   // 是否启用卡方检验
              fp32 chiSquareThreshold = 1e-8f); // 卡方检验阈值, 大于此值时忽略加速度观测
```

### 公开方法

| 方法 | 说明 |
|---|---|
| `reset()` | 重置滤波器状态、四元数、欧拉角 |
| `init()` | 初始化 KF 矩阵（由基类在首次 `update()` 时自动调用） |

> 继承自 AHRS 基类的方法：`update()`, `getEulerAngle()`, `getQuaternion()`, `getGyro()`, `getAccel()`, `getMotionAccelBodyFrame()`, `getMotionAccelEarthFrame()`

### 初始化流程

首次调用 `update()` 时，基类自动完成以下步骤（无需手动调用）：

1. `initQuaternion()` — 根据当前加速度计（和磁力计，如有）数据初始化四元数
2. `init()` — 初始化 EKF 矩阵，将已初始化的四元数写入状态向量

## 注意事项

1. 本算法进行了大量线性代数运算（Eigen 库），对**栈内存要求较高**。使用 FreeRTOS 分配 task 时，请务必确保分配了足够的栈内存。**建议分配 512 字 (2K 字节) 栈内存**。
2. 卡方检验阈值默认为 `1e-8`，可通过构造函数 `chiSquareThreshold` 参数调整。
3. `lambda` 参数用于防止零偏协方差过度收敛，建议值范围 (0, 1]，默认值 `1.0f` 表示不衰减。典型值为 `0.9996`。
4. `accLpfCoef` 参数为加速度一阶低通滤波的时间常数（单位：秒），值越大滤波效果越弱，`0` 表示不滤波。

## 典型实现代码

```cpp
// 初始化卡尔曼姿态解算算法
QuaternionEKF ahrs(
    0.0f,    // sampleFreq: 给定刷新频率, 给0则为DWT自动识别
    10.0f,   // quatProcessNoise: 四元数过程噪声基准
    0.001f,  // biasProcessNoise: 陀螺仪零偏过程噪声基准
    1e6f,    // measNoise: 加速度计量测噪声基准
    0.9f,    // lambda: 衰减系数
    0.0f,    // accLpfCoef: 加速度计LPF时间常数, 0表示不滤波
    true,    // isCheckChiSquare: 启用卡方检验
    1e-8f    // chiSquareThreshold: 卡方检验阈值
);

// 初始化IMU校准信息
BMI088::CalibrationInfo cali = {
    {0.0f, 0.0f, 0.0f},  // gyroOffset
    {0.0f, 0.0f, 0.0f},  // accelOffset
    {0.0f, 0.0f, 0.0f},  // magnetOffset
    {GSRLMath::Matrix33f::MatrixType::IDENTITY}
};

// 初始化IMU
BMI088 imu(&ahrs, {&hspi1, GPIOA, GPIO_PIN_4}, {&hspi1, GPIOB, GPIO_PIN_0}, cali);

// 主循环中调用
while (1) {
    GSRLMath::Vector3f eulerAngle = imu.solveAttitude();
}
```

## 参考/学习文件

1. [四元数EKF姿态更新算法](https://zhuanlan.zhihu.com/p/454155643)
2. [四元数解算姿态角解析](https://blog.csdn.net/guanjianhe/article/details/95608801)
3. [惯导姿态解算项目 (RoboMaster C Board INS Example)](https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example)