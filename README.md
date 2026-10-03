# QMI8658

QST QMI8658 6 轴惯性测量单元（IMU）的 SPI 驱动模块，带 PWM 恒温加热 / SPI driver module for the QST QMI8658 6-axis Inertial Measurement Unit (IMU) with PWM heater temperature control

## 1. 模块作用 / Purpose

构造时，QMI8658 将加热 PWM 设为 30 kHz、占空比 0 并使能，软复位芯片，等待 `WHO_AM_I` 读到 `0x05`（失败时每 100 ms 重试），然后按 `Param` 配置加速度计和陀螺仪的量程、输出频率（两者相同）与低通滤波，并使能两者和 INT2 中断。

数据由中断驱动：INT2 中断记录两次中断的间隔并发起一次 14 字节 SPI 连续读（温度、加速度、角速度）；SPI 完成回调中解析数据，减去陀螺仪零偏，经 `rotation` 旋转后发布，并用 PID 根据芯片温度计算加热 PWM 占空比（限制在 0 到 1）。默认 `pid_param` 的 `p`、`i`、`d` 均为 0，占空比保持 0，即加热关闭；`pid_param` 设置后按 `target_temperature` 恒温。

Upon construction, QMI8658 sets the heater PWM to 30 kHz with 0 duty and enables it, soft-resets the chip, waits for `WHO_AM_I` to read `0x05` (retrying every 100 ms), then configures the accelerometer and gyroscope range, the output data rate (the same for both) and the low-pass filters from `Param`, and enables both sensors and the INT2 interrupt.

Data acquisition is interrupt driven. The INT2 interrupt records the interval since the previous interrupt and starts a 14-byte SPI burst read (temperature, acceleration, angular rate). The SPI completion callback parses the data, subtracts the gyroscope offset, rotates both vectors by `rotation`, publishes them, and runs the heater PID on the chip temperature to set the PWM duty cycle (clamped to 0 to 1). The default `pid_param` has `p`, `i` and `d` equal to 0, so the duty cycle stays 0 and the heater is off; once `pid_param` is set, the chip temperature is regulated to `target_temperature`.

## 2. 陀螺仪零偏校准 / Gyroscope Calibration

零偏保存在 `database` 的键 `qmi8658_gyro_offset` 中，单位 rad/s。模块向 `ramfs` 添加命令 `qmi8658`：

```sh
qmi8658 show <time_ms> <interval_ms>  # 周期打印加速度、角速度和温度，interval 限制在 2 到 1000 ms / print acceleration, angular rate and temperature periodically, interval clamped to 2 to 1000 ms
qmi8658 list_offset                   # 打印当前零偏 / print the current offset
qmi8658 cali                          # 零偏校准 / calibrate the offset
```

`cali` 在设备静止时执行：清零零偏，等待 3 s，采集 60 s 求平均得到零偏，再采集 60 s 输出残余误差，最后把零偏写入 `database`。

The offset is stored under the `database` key `qmi8658_gyro_offset` in rad/s. The module adds the command `qmi8658` to `ramfs`.

`cali` runs with the device standing still: it clears the offset, waits 3 s, averages 60 s of samples to get the offset, samples another 60 s to print the residual error, and then saves the offset to `database`.

## 3. 构造接口 / Constructor

```cpp
struct Param
{
  ODR output_freq;
  GyroRange gyro_range;
  AcclRange accl_range;
  ModeLPF accl_lpf;
  ModeLPF gyro_lpf;
  LibXR::Quaternion<float> rotation;
  LibXR::PID<float>::Param pid_param;
  const char* gyro_topic_name;
  const char* accl_topic_name;
  float target_temperature;
};

QMI8658(LibXR::GPIO& int_pin2, LibXR::GPIO& cs_pin, LibXR::SPI& spi,
        LibXR::PWM& pwm, LibXR::Database& database, LibXR::RamFS& ramfs,
        const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `int_pin2`：连接 INT2 引脚的中断 GPIO（数据就绪）。
- `cs_pin`：SPI 片选输出 GPIO。
- `spi`：芯片所在的 SPI 总线。
- `pwm`：驱动加热电阻的 PWM 输出。
- `database`：保存陀螺仪零偏的数据库。
- `ramfs`：接收 `qmi8658` 命令的 RamFS。

配置参数（`Param`）：

- `output_freq`：加速度计和陀螺仪的输出频率 `QMI8658::ODR`，从 `ODR_7174_4HZ` 到 `ODR_28_025HZ`，默认 `ODR_896_8HZ`。
- `gyro_range`：陀螺仪量程 `QMI8658::GyroRange`，从 `DEG_16DPS` 到 `DEG_2048DPS`，默认 `DEG_2048DPS`。
- `accl_range`：加速度计量程 `QMI8658::AcclRange`，从 `ACCL_2G` 到 `ACCL_16G`，默认 `ACCL_16G`。
- `accl_lpf`、`gyro_lpf`：低通滤波 `QMI8658::ModeLPF`，可取 `LPF_2_66`、`LPF_3_63`、`LPF_5_39`、`LPF_13_37` 或 `LFP_DISABLE`，默认 `LFP_DISABLE`。
- `rotation`：安装姿态四元数，分量顺序 `(w, x, y, z)`，默认单位四元数。
- `pid_param`：加热 PID 参数 `LibXR::PID<float>::Param`（字段 `k, p, i, d, i_limit, out_limit, cycle`），默认 `k = 1.0`，`p`、`i`、`d`、`i_limit`、`out_limit` 为 0，`cycle = false`。
- `gyro_topic_name`、`accl_topic_name`：发布的 Topic 名称，默认 `qmi8658_gyro`、`qmi8658_accl`。
- `target_temperature`：目标芯片温度，单位 ℃，默认 45。

Dependencies:

- `int_pin2`: the interrupt GPIO connected to the INT2 pin (data ready).
- `cs_pin`: the SPI chip-select output GPIO.
- `spi`: the SPI bus the chip is on.
- `pwm`: the PWM output that drives the heater.
- `database`: the database that stores the gyroscope offset.
- `ramfs`: the RamFS that receives the `qmi8658` command.

Configuration parameters (`Param`):

- `output_freq`: output data rate of both sensors, `QMI8658::ODR`, from `ODR_7174_4HZ` to `ODR_28_025HZ`, default `ODR_896_8HZ`.
- `gyro_range`: gyroscope range, `QMI8658::GyroRange`, from `DEG_16DPS` to `DEG_2048DPS`, default `DEG_2048DPS`.
- `accl_range`: accelerometer range, `QMI8658::AcclRange`, from `ACCL_2G` to `ACCL_16G`, default `ACCL_16G`.
- `accl_lpf`, `gyro_lpf`: low-pass filter, `QMI8658::ModeLPF`, one of `LPF_2_66`, `LPF_3_63`, `LPF_5_39`, `LPF_13_37` or `LFP_DISABLE`, default `LFP_DISABLE`.
- `rotation`: mounting rotation quaternion with components in the order `(w, x, y, z)`, identity by default.
- `pid_param`: heater PID parameters, `LibXR::PID<float>::Param` (fields `k, p, i, d, i_limit, out_limit, cycle`), default `k = 1.0`, `p`, `i`, `d`, `i_limit` and `out_limit` equal to 0, `cycle = false`.
- `gyro_topic_name`, `accl_topic_name`: names of the published Topics, defaults `qmi8658_gyro`, `qmi8658_accl`.
- `target_temperature`: target chip temperature in °C, default 45.

## 4. Topic

| Topic（默认名称） | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `qmi8658_accl` | 发布 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 g，已按 `rotation` 旋转 |
| `qmi8658_gyro` | 发布 | `Eigen::Matrix<float, 3, 1>` | 角速度，已减去零偏并按 `rotation` 旋转，单位 rad/s |

| Topic (default name) | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `qmi8658_accl` | Publish | `Eigen::Matrix<float, 3, 1>` | Acceleration in g, rotated by `rotation` |
| `qmi8658_gyro` | Publish | `Eigen::Matrix<float, 3, 1>` | Angular rate in rad/s with the offset subtracted, rotated by `rotation` |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/QMI8658` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/QMI8658`, with the dependencies set to names registered by the BSP's `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/QMI8658
    id: qmi8658
    args:
      - int_pin2: IMU_INT
      - cs_pin: ACCL_CS
      - spi: spi1
      - pwm: pwm_tim10_ch1
      - database: database
      - ramfs: ramfs
      - param:
          output_freq: QMI8658::ODR::ODR_896_8HZ
          gyro_range: QMI8658::GyroRange::DEG_2048DPS
          accl_range: QMI8658::AcclRange::ACCL_16G
          accl_lpf: QMI8658::ModeLPF::LFP_DISABLE
          gyro_lpf: QMI8658::ModeLPF::LFP_DISABLE
          rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
          pid_param:
            k: 1.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: false
          gyro_topic_name: "qmi8658_gyro"
          accl_topic_name: "qmi8658_accl"
          target_temperature: 45
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片 QMI8658 IMU，通过 SPI 连接，INT2 引脚接到可触发中断的 GPIO，片选使用一个输出 GPIO；加热电阻由一路 PWM 输出驱动。

Dependencies: LibXR.

Hardware: one QMI8658 IMU on SPI, with the INT2 pin wired to a GPIO that can raise interrupts, one output GPIO as chip select, and the heater resistor driven by a PWM output.
