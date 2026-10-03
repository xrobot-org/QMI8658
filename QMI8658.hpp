#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: QST QMI8658 6 轴惯性测量单元（IMU）的 SPI 驱动模块，带 PWM 恒温加热 / SPI driver module for the QST QMI8658 6-axis Inertial Measurement Unit (IMU) with PWM heater temperature control
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "database.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "ramfs.hpp"
#include "semaphore.hpp"
#include "spi.hpp"
#include "thread.hpp"
#include "transform.hpp"

#define QMI8658_WHO_AM_I 0x00
#define QMI8658_REVISION 0x01

/* Setup and control registers */
#define QMI8658_CTRL1 0x02
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL4 0x05
#define QMI8658_CTRL5 0x06
#define QMI8658_CTRL6 0x07
#define QMI8658_CTRL7 0x08
#define QMI8658_CTRL9 0x0A

/* Data output registers */
#define QMI8658_TEMP_L 0x33
#define QMI8658_TEMP_H 0x34
#define QMI8658_ACC_X_L 0x35
#define QMI8658_ACC_X_H 0x36
#define QMI8658_ACC_Y_L 0x37
#define QMI8658_ACC_Y_H 0x38
#define QMI8658_ACC_Z_L 0x39
#define QMI8658_ACC_Z_H 0x3A
#define QMI8658_GYR_X_L 0x3B
#define QMI8658_GYR_X_H 0x3C
#define QMI8658_GYR_Y_L 0x3D
#define QMI8658_GYR_Y_H 0x3E
#define QMI8658_GYR_Z_L 0x3F
#define QMI8658_GYR_Z_H 0x40

/* Soft reset register */
#define QMI8658_RESET 0x60

/* read burst length from TEMP_L to GYR_Z_H (14 bytes) */
#define QMI8658_READ_LEN 14

/**
 * @brief QMI8658 6 轴 IMU 驱动，通过 SPI 读取并发布加速度与角速度，用 PWM 加热恒温。
 *        Driver for the QMI8658 6-axis IMU; reads it over SPI, publishes acceleration
 *        and angular rate, and regulates the chip temperature with a PWM heater.
 */
class QMI8658
{
 public:
  /// 角度转弧度系数 Degree to radian factor
  static constexpr float M_DEG2RAD_MULT = 0.01745329251f;

  /**
   * @brief 加速度计与陀螺仪的输出频率。
   *        Output data rate of the accelerometer and gyroscope.
   */
  enum class ODR : uint8_t
  {
    ODR_7174_4HZ = 0,  ///< 7174.4 Hz
    ODR_3587_2HZ,      ///< 3587.2 Hz
    ODR_1793_6HZ,      ///< 1793.6 Hz
    ODR_896_8HZ,       ///< 896.8 Hz
    ODR_448_4HZ,       ///< 448.4 Hz
    ODR_224_2HZ,       ///< 224.2 Hz
    ODR_112_1HZ,       ///< 112.1 Hz
    ODR_56_05HZ,       ///< 56.05 Hz
    ODR_28_025HZ       ///< 28.025 Hz
  };

  /**
   * @brief 加速度计量程。
   *        Accelerometer range.
   */
  enum class AcclRange : uint8_t
  {
    ACCL_2G = 0,  ///< ±2 g
    ACCL_4G,      ///< ±4 g
    ACCL_8G,      ///< ±8 g
    ACCL_16G      ///< ±16 g
  };

  /**
   * @brief 陀螺仪量程。
   *        Gyroscope range.
   */
  enum class GyroRange : uint8_t
  {
    DEG_16DPS = 0,  ///< ±16 °/s
    DEG_32DPS,      ///< ±32 °/s
    DEG_64DPS,      ///< ±64 °/s
    DEG_128DPS,     ///< ±128 °/s
    DEG_256DPS,     ///< ±256 °/s
    DEG_512DPS,     ///< ±512 °/s
    DEG_1024DPS,    ///< ±1024 °/s
    DEG_2048DPS     ///< ±2048 °/s
  };

  /**
   * @brief 低通滤波模式，枚举值等于寄存器取值。
   *        Low-pass filter mode; the enumerator value equals the register value.
   */
  enum class ModeLPF : uint8_t
  {
    LPF_2_66 = 1,    ///< 带宽 2.66% ODR Bandwidth 2.66% of ODR
    LPF_3_63 = 3,    ///< 带宽 3.63% ODR Bandwidth 3.63% of ODR
    LPF_5_39 = 5,    ///< 带宽 5.39% ODR Bandwidth 5.39% of ODR
    LPF_13_37 = 7,   ///< 带宽 13.37% ODR Bandwidth 13.37% of ODR
    LFP_DISABLE = 0  ///< 关闭低通滤波 Low-pass filter disabled
  };

#pragma pack(push, 1)
  /**
   * @brief 从 TEMP_L 起连续读出的 14 字节原始数据。
   *        The 14 raw bytes read in one burst starting at TEMP_L.
   */
  struct RegRawData
  {
    uint16_t temp;    ///< 温度原始值，256 LSB/℃ Raw temperature, 256 LSB/°C
    int16_t accl[3];  ///< 加速度原始值 x、y、z Raw acceleration x, y, z
    int16_t gyro[3];  ///< 角速度原始值 x、y、z Raw angular rate x, y, z
  };
#pragma pack(pop)

  /**
   * @brief 构造参数。
   *        Construction parameters.
   */
  struct Param
  {
    ODR output_freq;       ///< 加速度计与陀螺仪输出频率 Output data rate of both sensors
    GyroRange gyro_range;  ///< 陀螺仪量程 Gyroscope range
    AcclRange accl_range;  ///< 加速度计量程 Accelerometer range
    ModeLPF accl_lpf;      ///< 加速度计低通滤波 Accelerometer low-pass filter
    ModeLPF gyro_lpf;      ///< 陀螺仪低通滤波 Gyroscope low-pass filter
    LibXR::Quaternion<float> rotation;   ///< 安装姿态 (w,x,y,z) Mounting rotation
    LibXR::PID<float>::Param pid_param;  ///< 加热 PID 参数 Heater PID parameters
    const char* gyro_topic_name;         ///< 角速度 Topic 名称 Angular-rate Topic name
    const char* accl_topic_name;         ///< 加速度 Topic 名称 Acceleration Topic name
    float target_temperature;            ///< 目标芯片温度，℃ Target chip temperature, °C
  };

  /**
   * @brief 构造 QMI8658：配置加热 PWM，复位并配置芯片，使能 INT2 中断，注册命令。
   *        Construct QMI8658: configure the heater PWM, reset and configure the chip,
   *        enable the INT2 interrupt and register the `qmi8658` command.
   *
   * @param int_pin2 连接 INT2 引脚的中断 GPIO。
   *                 Interrupt GPIO connected to the INT2 pin.
   * @param cs_pin SPI 片选输出 GPIO。
   *               SPI chip-select output GPIO.
   * @param spi 芯片所在的 SPI 总线。
   *            SPI bus of the chip.
   * @param pwm 驱动加热电阻的 PWM 输出。
   *            PWM output that drives the heater.
   * @param database 保存陀螺仪零偏的数据库。
   *                 Database that stores the gyroscope offset.
   * @param ramfs 接收 `qmi8658` 命令的 RamFS。
   *              RamFS that receives the `qmi8658` command.
   * @param param 构造参数。
   *              Construction parameters.
   */
  QMI8658(LibXR::GPIO& int_pin2, LibXR::GPIO& cs_pin, LibXR::SPI& spi, LibXR::PWM& pwm,
          LibXR::Database& database, LibXR::RamFS& ramfs,
          const Param& param = {.output_freq = QMI8658::ODR::ODR_896_8HZ,
                                .gyro_range = QMI8658::GyroRange::DEG_2048DPS,
                                .accl_range = QMI8658::AcclRange::ACCL_16G,
                                .accl_lpf = QMI8658::ModeLPF::LFP_DISABLE,
                                .gyro_lpf = QMI8658::ModeLPF::LFP_DISABLE,
                                .rotation = {1.0f, 0.0f, 0.0f, 0.0f},
                                .pid_param = {.k = 1.0f,
                                              .p = 0.0f,
                                              .i = 0.0f,
                                              .d = 0.0f,
                                              .i_limit = 0.0f,
                                              .out_limit = 0.0f,
                                              .cycle = false},
                                .gyro_topic_name = "qmi8658_gyro",
                                .accl_topic_name = "qmi8658_accl",
                                .target_temperature = 45})
      : output_freq_(param.output_freq),
        gyro_range_(param.gyro_range),
        accel_range_(param.accl_range),
        accl_lpf_(param.accl_lpf),
        gyro_lpf_(param.gyro_lpf),
        rotation_(param.rotation),
        target_temperature_(param.target_temperature),
        topic_gyro_(
            LibXR::Topic::CreateTopic<decltype(gyro_data_)>(param.gyro_topic_name)),
        topic_accl_(
            LibXR::Topic::CreateTopic<decltype(accl_data_)>(param.accl_topic_name)),
        int_(std::addressof(int_pin2)),
        cs_(std::addressof(cs_pin)),
        spi_(std::addressof(spi)),
        pwm_(std::addressof(pwm)),
        pid_heat_(param.pid_param),
        op_spi_block_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("qmi8658", CommandFunc, this)),
        gyro_offset_key_(database, "qmi8658_gyro_offset",
                         Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f))
  {
    cs_->Write(true);
    int_->DisableInterrupt();

    // INT: 记录 dt 并发起一次 SPI 读
    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, QMI8658* self)
        {
          auto now = LibXR::Timebase::GetMicroseconds();
          self->dt_gyro_ = now - self->last_gyro_int_time_;
          self->last_gyro_int_time_ = now;
          self->ReadData(in_isr);
        },
        this);
    int_->RegisterCallback(int_cb);

    spi_cb_ = LibXR::SPI::OperationRW::Callback::Create(
        [](bool in_isr, QMI8658* self, LibXR::ErrorCode err)
        {
          self->cs_->Write(true);
          if (err == LibXR::ErrorCode::OK)
          {
            self->ParseData(in_isr);

            float dt_sec = std::max(1e-4f, self->dt_gyro_.ToSecondf());
            float duty = self->pid_heat_.Calculate(self->target_temperature_,
                                                   self->temperature_, dt_sec);
            duty = std::clamp(duty, 0.0f, 1.0f);
            self->pwm_->SetDutyCycle(duty);
          }
          self->reading_.store(false);
        },
        this);
    op_spi_cb_ = LibXR::SPI::OperationRW(spi_cb_);

    pwm_->SetConfig({.frequency = 30000});
    pwm_->SetDutyCycle(0.0f);
    pwm_->Enable();

    Init();
    int_->EnableInterrupt();

    ramfs.Add(cmd_file_);
  }

  /**
   * @brief 写入一个寄存器。
   *        Write one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @param data 写入值。
   *             Value to write.
   */
  void WriteSingle(uint8_t reg, uint8_t data)
  {
    cs_->Write(false);
    spi_->MemWrite(reg, data, op_spi_block_);
    cs_->Write(true);
  }

  /**
   * @brief 读取一个寄存器。
   *        Read one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @return 寄存器值。
   *         Register value.
   */
  uint8_t ReadSingle(uint8_t reg)
  {
    uint8_t res = 0;
    cs_->Write(false);
    spi_->MemRead(reg, res, op_spi_block_);
    cs_->Write(true);
    return res;
  }

  /**
   * @brief 发起一次从 TEMP_L 起的 14 字节异步连续读，完成后由 SPI 回调解析并释放片选；
   *        上一次连续读未完成时直接返回。
   *        Start an asynchronous 14-byte burst read from TEMP_L; the SPI callback parses
   *        it and releases the chip select on completion. Returns at once while the
   *        previous burst read is still in flight.
   *
   * @param in_isr 是否在中断上下文中调用。
   *               Whether called from interrupt context.
   */
  void ReadData(bool in_isr = false)
  {
    // 上一次连续读尚未完成时丢弃本次中断，片选保持不变
    // Drop this interrupt while the previous burst read is in flight; the chip select
    // stays as it is
    bool idle = false;
    if (!reading_.compare_exchange_strong(idle, true))
    {
      return;
    }
    cs_->Write(false);
    // 连续读：MemRead 自动补读位(0x80)，一次异步传输读 14 字节到结构体
    auto ans = spi_->MemRead(QMI8658_TEMP_L, rw_buffer_, op_spi_cb_, in_isr);
    if (ans != LibXR::ErrorCode::OK)
    {
      // 传输未启动，不会有回调，在此释放片选
      cs_->Write(true);
      reading_.store(false);
    }
  }

  /**
   * @brief 软复位芯片，等待 WHO_AM_I 为 0x05，再写入量程、频率、滤波和使能配置。
   *        Soft-reset the chip, wait for WHO_AM_I to read 0x05, then write the range,
   *        data rate, filter and enable configuration.
   */
  void Init()
  {
    WriteSingle(QMI8658_RESET, 0x80);
    LibXR::Thread::Sleep(200);

    while (ReadSingle(QMI8658_WHO_AM_I) != 0x05)
    {
      uint8_t data[128];
      for (int i = 0; i < 128; i++) data[i] = ReadSingle(i);
      XR_LOG_ERROR("QMI8658 not found:%u, retrying...",
                   static_cast<unsigned>(data[QMI8658_WHO_AM_I]));
      LibXR::Thread::Sleep(100);
    }

    // 6: ADDR_AI=1 | 5: BE=0 | 4: INT2EN=1 | 3: INT1EN=1 | 2: FIFO_INT_SEL=1
    WriteSingle(QMI8658_CTRL1, 0x74);

    // 6-4: ACC_SCL | 3:0: ACC_ODR
    WriteSingle(QMI8658_CTRL2, (static_cast<uint8_t>(accel_range_) << 4) |
                                   static_cast<uint8_t>(output_freq_));

    // 6-4: GYRO_SCL | 3:0: GYRO_ODR
    WriteSingle(QMI8658_CTRL3, (static_cast<uint8_t>(gyro_range_) << 4) |
                                   static_cast<uint8_t>(output_freq_));

    // 6-4: GYRO_LPF_MODE | 2-0: ACCL_LPF_MODE
    WriteSingle(QMI8658_CTRL5,
                (static_cast<uint8_t>(gyro_lpf_) << 4) | static_cast<uint8_t>(accl_lpf_));

    // 7: SyncSample=true | 5: DRDY_DIS=0 | 4: gSN=0 | 1: gEN=1 | 0: aEN=1
    WriteSingle(QMI8658_CTRL7, 0x03);
  }

  /**
   * @brief 当前加速度计量程对应的分辨率。
   *        Resolution for the current accelerometer range.
   *
   * @return 每 LSB 对应的加速度，单位 g。
   *         Acceleration per LSB in g.
   */
  float GetAccelLSB()
  {
    switch (accel_range_)
    {
      case AcclRange::ACCL_2G:
        return 2.0f / 32768.0f;
      case AcclRange::ACCL_4G:
        return 4.0f / 32768.0f;
      case AcclRange::ACCL_8G:
        return 8.0f / 32768.0f;
      case AcclRange::ACCL_16G:
        return 16.0f / 32768.0f;
      default:
        return 0.0f;
    }
  }

  /**
   * @brief 当前陀螺仪量程对应的分辨率。
   *        Resolution for the current gyroscope range.
   *
   * @return 每 LSB 对应的角速度，单位 rad/s。
   *         Angular rate per LSB in rad/s.
   */
  float GetGyroLSB()
  {
    switch (gyro_range_)
    {
      case GyroRange::DEG_16DPS:
        return 16.0f / 32768.0f * M_DEG2RAD_MULT;
      case GyroRange::DEG_32DPS:
        return 32.0f / 32768.0f * M_DEG2RAD_MULT;
      case GyroRange::DEG_64DPS:
        return 64.0f / 32768.0f * M_DEG2RAD_MULT;
      case GyroRange::DEG_128DPS:
        return 128.0f / 32768.0f * M_DEG2RAD_MULT;
      case GyroRange::DEG_256DPS:
        return 256.0f / 32768.0f * M_DEG2RAD_MULT;
      case GyroRange::DEG_512DPS:
        return 512.0f / 32768.0f * M_DEG2RAD_MULT;
      case GyroRange::DEG_1024DPS:
        return 1024.0f / 32768.0f * M_DEG2RAD_MULT;
      case GyroRange::DEG_2048DPS:
        return 2048.0f / 32768.0f * M_DEG2RAD_MULT;
      default:
        return 0.0f;
    }
  }

  /**
   * @brief 解析一次连续读的数据，减去陀螺仪零偏，按 rotation 旋转并发布。
   *        Parse one burst read, subtract the gyroscope offset, rotate by rotation and
   *        publish.
   *
   * @param in_isr 是否在中断上下文中调用。
   *               Whether called from interrupt context.
   */
  void ParseData(bool in_isr)
  {
    temperature_ = static_cast<float>(static_cast<uint16_t>(rw_buffer_.temp)) / 256.0f;

    Eigen::Vector3f accl_raw(rw_buffer_.accl[0], rw_buffer_.accl[1], rw_buffer_.accl[2]);
    Eigen::Vector3f gyro_raw(rw_buffer_.gyro[0], rw_buffer_.gyro[1], rw_buffer_.gyro[2]);

    accl_raw *= GetAccelLSB();
    Eigen::Vector3f gyro = gyro_raw * GetGyroLSB();

    if (in_cali_)
    {
      gyro_cali_.data()[0] += static_cast<int64_t>(rw_buffer_.gyro[0]);
      gyro_cali_.data()[1] += static_cast<int64_t>(rw_buffer_.gyro[1]);
      gyro_cali_.data()[2] += static_cast<int64_t>(rw_buffer_.gyro[2]);
      cali_counter_++;
    }

    gyro -= gyro_offset_key_.data_;

    accl_data_ = rotation_ * accl_raw;
    gyro_data_ = rotation_ * gyro;

    topic_accl_.PublishFromCallback(accl_data_, in_isr);
    topic_gyro_.PublishFromCallback(gyro_data_, in_isr);
  }

  /**
   * @brief `qmi8658` 命令入口：`show`、`list_offset` 与 `cali`。
   *        Entry of the `qmi8658` command: `show`, `list_offset` and `cali`.
   *
   * @param self QMI8658 实例。
   *             QMI8658 instance.
   * @param argc 参数个数。
   *             Argument count.
   * @param argv 参数列表。
   *             Argument list.
   * @return 成功为 0，参数无效为 -1。
   *         0 on success, -1 for invalid arguments.
   */
  static int CommandFunc(QMI8658* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n">();
      LibXR::STDIO::Printf<
          "  list_offset                  - Show current gyro "
          "calibration offset.\r\n">();
      LibXR::STDIO::Printf<
          "  cali                         - Start gyroscope calibration.\r\n">();
    }
    else if (argc == 2)
    {
      if (std::strcmp(argv[1], "list_offset") == 0)
      {
        LibXR::STDIO::Printf<"Current calibration offset - x: %f, y: %f, z: %f\r\n">(
            self->gyro_offset_key_.data_.x(), self->gyro_offset_key_.data_.y(),
            self->gyro_offset_key_.data_.z());
      }
      else if (std::strcmp(argv[1], "cali") == 0)
      {
        // 第一次：采集 60s 计算偏置
        self->gyro_offset_key_.data_.setZero();
        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0, 0, 0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;

        LibXR::STDIO::Printf<
            "Starting gyroscope calibration. Please keep the "
            "device steady.\r\n">();
        LibXR::Thread::Sleep(3000);
        for (int i = 0; i < 60; ++i)
        {
          LibXR::STDIO::Printf<"Progress: %d / 60\r\n">(i + 1);
          LibXR::Thread::Sleep(1000);
        }
        self->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        double denom = static_cast<double>(self->cali_counter_);
        if (denom < 1.0) denom = 1.0;
        self->gyro_offset_key_.data_.x() =
            static_cast<double>(self->gyro_cali_.data()[0]) / denom * self->GetGyroLSB();
        self->gyro_offset_key_.data_.y() =
            static_cast<double>(self->gyro_cali_.data()[1]) / denom * self->GetGyroLSB();
        self->gyro_offset_key_.data_.z() =
            static_cast<double>(self->gyro_cali_.data()[2]) / denom * self->GetGyroLSB();

        LibXR::STDIO::Printf<"Calibration result - x: %f, y: %f, z: %f\r\n">(
            self->gyro_offset_key_.data_.x(), self->gyro_offset_key_.data_.y(),
            self->gyro_offset_key_.data_.z());

        // 第二次：采集 60s 评估误差
        LibXR::STDIO::Printf<"Analyzing calibration quality...\r\n">();
        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0, 0, 0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;
        for (int i = 0; i < 60; ++i)
        {
          LibXR::STDIO::Printf<"Progress: %d / 60\r\n">(i + 1);
          LibXR::Thread::Sleep(1000);
        }
        self->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        double denom2 = static_cast<double>(self->cali_counter_);
        if (denom2 < 1.0) denom2 = 1.0;

        const float mean_x =
            static_cast<double>(self->gyro_cali_.data()[0]) / denom2 * self->GetGyroLSB();
        const float mean_y =
            static_cast<double>(self->gyro_cali_.data()[1]) / denom2 * self->GetGyroLSB();
        const float mean_z =
            static_cast<double>(self->gyro_cali_.data()[2]) / denom2 * self->GetGyroLSB();

        LibXR::STDIO::Printf<"Calibration error - x: %f, y: %f, z: %f\r\n">(
            mean_x - self->gyro_offset_key_.data_.x(),
            mean_y - self->gyro_offset_key_.data_.y(),
            mean_z - self->gyro_offset_key_.data_.z());

        self->gyro_offset_key_.Set(self->gyro_offset_key_.data_);
        LibXR::STDIO::Printf<"Calibration data saved.\r\n">();
      }
    }
    else if (argc == 4)
    {
      if (std::strcmp(argv[1], "show") == 0)
      {
        int time = std::atoi(argv[2]);
        int delay = std::atoi(argv[3]);
        delay = std::clamp(delay, 2, 1000);

        while (time > 0)
        {
          LibXR::STDIO::Printf<
              "Accel: x = %+5f, y = %+5f, z = %+5f | Gyro: x "
              "= %+5f, y = %+5f, z = %+5f "
              "| Temp: %+5f\r\n">(self->accl_data_.x(), self->accl_data_.y(),
                                  self->accl_data_.z(), self->gyro_data_.x(),
                                  self->gyro_data_.y(), self->gyro_data_.z(),
                                  self->temperature_);
          LibXR::Thread::Sleep(delay);
          time -= delay;
        }
      }
    }
    else
    {
      LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
      return -1;
    }
    return 0;
  }

 private:
  int16_t SwitchByte(int16_t val) { return (val << 8) | (val >> 8); }

  ODR output_freq_ = ODR::ODR_896_8HZ;
  GyroRange gyro_range_ = GyroRange::DEG_2048DPS;
  AcclRange accel_range_ = AcclRange::ACCL_16G;
  ModeLPF accl_lpf_ = ModeLPF::LFP_DISABLE;
  ModeLPF gyro_lpf_ = ModeLPF::LFP_DISABLE;

  LibXR::Quaternion<float> rotation_;

  bool in_cali_ = false;
  uint32_t cali_counter_ = 0;
  Eigen::Matrix<int64_t, 3, 1> gyro_cali_;
  float temperature_ = 0.0f;

  LibXR::MicrosecondTimestamp last_gyro_int_time_ = 0;
  LibXR::MicrosecondTimestamp::Duration dt_gyro_ = 0;

  float target_temperature_ = 25.0f;

  RegRawData rw_buffer_{};
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;
  LibXR::Topic topic_gyro_, topic_accl_;
  LibXR::GPIO* int_ = nullptr;
  LibXR::GPIO* cs_ = nullptr;
  LibXR::SPI* spi_ = nullptr;
  LibXR::PWM* pwm_ = nullptr;
  LibXR::PID<float> pid_heat_;
  LibXR::Semaphore sem_spi_;
  LibXR::SPI::OperationRW::Callback spi_cb_;
  LibXR::SPI::OperationRW op_spi_block_, op_spi_cb_;
  std::atomic<bool> reading_{false};  ///< 异步连续读进行中 Burst read in flight

  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_offset_key_;
};
