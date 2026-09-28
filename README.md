# QMI8658

上海矽睿科技有限公司（QST）QMI8658 6 轴惯性测量单元（IMU）的 SPI 驱动模块，带 PWM 恒温加热。
SPI driver module for the QST QMI8658 6-axis Inertial Measurement Unit (IMU),
with PWM heater temperature control.

构造时模块将加热 PWM 设为 30 kHz、占空比 0 并使能，软复位芯片，等待 `WHO_AM_I`
读到 `0x05`（失败时每 100 ms 重试），然后按 `Param` 配置加速度计和陀螺仪的量程、
输出频率（两者相同）与低通滤波，并使能两者和 INT2 中断。

模块没有线程，数据由中断驱动：INT2 中断记录两次中断的间隔并发起一次 14 字节
SPI 连续读（温度、加速度、角速度）；SPI 完成回调中解析数据，减去陀螺仪零偏，
经 `rotation` 旋转后发布，并用 PID 根据芯片温度计算加热 PWM 占空比（限制在 0..1）。

During construction the module sets the heater PWM to 30 kHz with 0 duty and
enables it, soft-resets the chip, waits for `WHO_AM_I` to read `0x05`
(retrying every 100 ms), then configures accelerometer and gyroscope range,
output data rate (the same for both) and low-pass filters from `Param`, and
enables both sensors and the INT2 interrupt.

The module has no thread; it is interrupt driven. The INT2 interrupt records
the interval since the previous interrupt and starts a 14-byte SPI burst read
(temperature, acceleration, angular rate). The SPI completion callback parses
the data, subtracts the gyroscope offset, rotates both vectors by `rotation`,
publishes them, and runs the heater PID on the chip temperature to set the PWM
duty cycle (clamped to 0..1).

| Topic（默认 / default） | 类型 / Type | 内容 / Content |
| --- | --- | --- |
| `qmi8658_accl` | `Eigen::Matrix<float, 3, 1>` | 加速度，g / acceleration in g |
| `qmi8658_gyro` | `Eigen::Matrix<float, 3, 1>` | 角速度（已减零偏），rad/s / angular rate (offset removed) in rad/s |

使用默认 `pid_param`（增益全为 0）时占空比始终为 0，即不加热；需要恒温时请设置 PID 参数。
With the default `pid_param` (all gains zero) the duty cycle stays 0, i.e. the
heater is off; set the PID gains to enable temperature control.

### 陀螺仪零偏校准 / Gyroscope calibration

零偏保存在 `database` 的键 `qmi8658_gyro_offset` 中（rad/s）。模块在 `ramfs` 中注册命令 `qmi8658`：
The offset is stored under the `database` key `qmi8658_gyro_offset` (rad/s).
The module adds the command `qmi8658` to `ramfs`:

```sh
qmi8658 show <time_ms> <interval_ms>  # 周期打印加速度、角速度和温度（interval 限制在 2..1000 ms）/ print accel, gyro and temperature (interval clamped to 2..1000 ms)
qmi8658 list_offset                   # 打印当前零偏 / print the current offset
qmi8658 cali                          # 零偏校准 / calibrate the offset
```

`cali` 期间保持设备静止：清零零偏，等待 3 s，采集 60 s 求平均得到零偏，再采集 60 s
输出残余误差，最后写入 `database`。
Keep the device still during `cali`: it clears the offset, waits 3 s, averages
60 s of samples to get the offset, samples another 60 s to print the residual
error, then saves the offset to `database`.

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。
No other Modules; LibXR only.

## 构造接口 / Constructor

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
        const Param& param = {...});
```

依赖 / Dependencies:

- `int_pin2`：连接 INT2 引脚的中断 GPIO（数据就绪）。/ Interrupt GPIO connected to the INT2 pin (data ready).
- `cs_pin`：SPI 片选输出 GPIO。/ SPI chip-select output GPIO.
- `spi`：芯片所在的 SPI 总线。/ The SPI bus the chip is on.
- `pwm`：加热电阻的 PWM 输出。/ PWM output driving the heater.
- `database`：保存陀螺仪零偏的数据库。/ Database that stores the gyroscope offset.
- `ramfs`：注册 `qmi8658` 命令的 RamFS。/ RamFS that receives the `qmi8658` command.

配置 / Configuration (`Param`，括号内为默认值 / defaults in parentheses):

- `output_freq`：加速度计和陀螺仪的输出频率 `QMI8658::ODR`，`ODR_7174_4HZ` … `ODR_28_025HZ`（`ODR_896_8HZ`）。
  / Output data rate of both sensors, `QMI8658::ODR`, `ODR_7174_4HZ` … `ODR_28_025HZ` (`ODR_896_8HZ`).
- `gyro_range`：`QMI8658::GyroRange`，`DEG_16DPS` … `DEG_2048DPS`（`DEG_2048DPS`）。
- `accl_range`：`QMI8658::AcclRange`，`ACCL_2G` … `ACCL_16G`（`ACCL_16G`）。
- `accl_lpf`、`gyro_lpf`：`QMI8658::ModeLPF`，`LPF_2_66`、`LPF_3_63`、`LPF_5_39`、`LPF_13_37` 或 `LFP_DISABLE`（`LFP_DISABLE`）。
- `rotation`：安装姿态四元数 (w, x, y, z)（单位四元数）。/ Mounting rotation quaternion (w, x, y, z) (identity).
- `pid_param`：加热 PID 参数 `k, p, i, d, i_limit, out_limit, cycle`（`k = 1`，其余为 0 / `false`）。
  / Heater PID parameters (`k = 1`, the rest 0 / `false`).
- `gyro_topic_name`、`accl_topic_name`：Topic 名（`qmi8658_gyro`、`qmi8658_accl`）。/ Topic names.
- `target_temperature`：目标温度，°C（45）。/ Target chip temperature in °C (45).

## 使用 / Use

```sh
xrobot module add xrobot-org/QMI8658
xrobot setup
xrobot instance add xrobot-org/QMI8658
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of
objects the BSP registers with `XR_REGISTER`:

```yaml
modules:
  - module: xrobot-org/QMI8658
    id: qmi8658_0
    args:
      - int_pin2: imu_int2
      - cs_pin: imu_cs
      - spi: spi1
      - pwm: imu_heat_pwm
      - database: database
      - ramfs: ramfs
      - param:
          output_freq: QMI8658::ODR::ODR_896_8HZ
          gyro_range: QMI8658::GyroRange::DEG_2048DPS
          accl_range: QMI8658::AcclRange::ACCL_16G
          accl_lpf: QMI8658::ModeLPF::LFP_DISABLE
          gyro_lpf: QMI8658::ModeLPF::LFP_DISABLE
          rotation:
            - 1.0f
            - 0.0f
            - 0.0f
            - 0.0f
          pid_param:
            k: 1.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          gyro_topic_name: '"qmi8658_gyro"'
          accl_topic_name: '"qmi8658_accl"'
          target_temperature: '45'
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(imu_int2, LibXR::GPIO);
XR_REGISTER(imu_cs, LibXR::GPIO);
XR_REGISTER(spi1, LibXR::SPI);
XR_REGISTER(imu_heat_pwm, LibXR::PWM);
XR_REGISTER(database, LibXR::Database);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/QMI8658`
（在 BSP 中）打印当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/QMI8658` in a BSP, prints the current
constructor.
