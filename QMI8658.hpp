#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 上海矽睿科技有限公司 QMI8658 6 轴惯性测量单元（IMU）的驱动模块 / Driver module for the QMI8658 6-axis Inertial Measurement Unit (IMU) from QST Corporation Limited.
constructor_args:
  - output_freq: QMI8658::ODR::ODR_896_8HZ
  - gyro_range: QMI8658::GyroRange::DEG_2048DPS
  - accl_range: QMI8658::AcclRange::ACCL_16G
  - accl_lpf: QMI8658::ModeLPF::LFP_DISABLE
  - gyro_lpf: QMI8658::ModeLPF::LFP_DISABLE
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - pid_param:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - gyro_topic_name: "qmi8658_gyro"
  - accl_topic_name: "qmi8658_accl"
  - spi_name: "spi1"
  - cs_pin_name: "spi1_cs"
  - int_pin2_name: "spi1_int2"
  - pwm_name: "pwm1"
  - target_temperature: 45
template_args: []
required_hardware: spi_name cs_pin_name int_pin1_name int_pin2_name pwm_name ramfs database
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "semaphore.hpp"
#include "spi.hpp"
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

class QMI8658 : public LibXR::Application
{
 public:
  static constexpr float M_DEG2RAD_MULT = 0.01745329251f;

  enum class ODR : uint8_t
  {
    ODR_7174_4HZ = 0,
    ODR_3587_2HZ,
    ODR_1793_6HZ,
    ODR_896_8HZ,
    ODR_448_4HZ,
    ODR_224_2HZ,
    ODR_112_1HZ,
    ODR_56_05HZ,
    ODR_28_025HZ
  };

  enum class AcclRange : uint8_t
  {
    ACCL_2G = 0,
    ACCL_4G,
    ACCL_8G,
    ACCL_16G
  };

  enum class GyroRange : uint8_t
  {
    DEG_16DPS = 0,
    DEG_32DPS,
    DEG_64DPS,
    DEG_128DPS,
    DEG_256DPS,
    DEG_512DPS,
    DEG_1024DPS,
    DEG_2048DPS
  };

  enum class ModeLPF : uint8_t
  {
    LPF_2_66 = 1,
    LPF_3_63 = 3,
    LPF_5_39 = 5,
    LPF_13_37 = 7,
    LFP_DISABLE = 0
  };

#pragma pack(push, 1)
  struct RegRawData
  {
    uint16_t temp;
    int16_t accl[3];
    int16_t gyro[3];
  };
#pragma pack(pop)

  QMI8658(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, ODR output_freq,
          GyroRange gyro_range, AcclRange accl_range, ModeLPF accl_lpf, ModeLPF gyro_lpf,
          LibXR::Quaternion<float>&& rotation, LibXR::PID<float>::Param pid_param,
          const char* gyro_topic_name, const char* accl_topic_name, const char* spi_name,
          const char* cs_pin_name, const char* int_pin2_name, const char* pwm_name,
          float target_temperature)
      : output_freq_(output_freq),
        gyro_range_(gyro_range),
        accel_range_(accl_range),
        accl_lpf_(accl_lpf),
        gyro_lpf_(gyro_lpf),
        rotation_(rotation),
        target_temperature_(target_temperature),
        topic_gyro_(gyro_topic_name, sizeof(gyro_data_)),
        topic_accl_(accl_topic_name, sizeof(accl_data_)),
        int_(hw.template FindOrExit<LibXR::GPIO>({int_pin2_name})),
        cs_(hw.template FindOrExit<LibXR::GPIO>({cs_pin_name})),
        spi_(hw.template FindOrExit<LibXR::SPI>({spi_name})),
        pwm_(hw.template FindOrExit<LibXR::PWM>({pwm_name})),
        pid_heat_(pid_param),
        op_spi_block_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("qmi8658", CommandFunc, this)),
        gyro_offset_key_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                         "qmi8658_gyro_offset",
                         Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f))
  {
    cs_->Write(true);
    int_->DisableInterrupt();

    // INT: 记录 dt 并发起一次 SPI 读
    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, QMI8658* self)
        {
          (void)in_isr;
          auto now = LibXR::Timebase::GetMicroseconds();
          self->dt_gyro_ = now - self->last_gyro_int_time_;
          self->last_gyro_int_time_ = now;
          self->ReadData();
        },
        this);
    int_->RegisterCallback(int_cb);

    spi_cb_ = LibXR::SPI::OperationRW::Callback::Create(
        [](bool in_isr, QMI8658* self, ErrorCode err)
        {
          (void)in_isr;
          self->cs_->Write(true);
          if (err == ErrorCode::OK)
          {
            self->ParseData();

            float dt_sec = std::max(1e-4f, self->dt_gyro_.ToSecondf());
            float duty = self->pid_heat_.Calculate(self->target_temperature_,
                                                   self->temperature_, dt_sec);
            duty = std::clamp(duty, 0.0f, 1.0f);
            self->pwm_->SetDutyCycle(duty);
          }
        },
        this);
    op_spi_cb_ = LibXR::SPI::OperationRW(spi_cb_);

    pwm_->SetConfig({30000});
    pwm_->SetDutyCycle(0.0f);
    pwm_->Enable();

    Init();
    int_->EnableInterrupt();

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
    app.Register(*this);
  }

  void OnMonitor() override {}

  void WriteSingle(uint8_t reg, uint8_t data)
  {
    cs_->Write(false);
    spi_->MemWrite(reg, data, op_spi_block_);
    cs_->Write(true);
  }

  uint8_t ReadSingle(uint8_t reg)
  {
    uint8_t res = 0;
    cs_->Write(false);
    spi_->MemRead(reg, res, op_spi_block_);
    cs_->Write(true);
    return res;
  }

  void ReadData()
  {
    cs_->Write(false);
    // 连续读：先写起始寄存器 + 读位(0x80)，再读 14 字节到结构体
    uint8_t reg = QMI8658_TEMP_L | 0x80;
    spi_->Write(reg, op_spi_block_);
    spi_->Read(rw_buffer_, op_spi_cb_);
  }

  void Init()
  {
    WriteSingle(QMI8658_RESET, 0x80);
    LibXR::Thread::Sleep(200);

    while (ReadSingle(QMI8658_WHO_AM_I) != 0x05)
    {
      uint8_t data[128];
      for (int i = 0; i < 128; i++) data[i] = ReadSingle(i);
      XR_LOG_ERROR("QMI8658 not found:%d, retrying...", data[QMI8658_WHO_AM_I]);
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

  void ParseData()
  {
    temperature_ = static_cast<float>(static_cast<uint16_t>(rw_buffer_.temp)) / 256.0f;

    Eigen::Vector3f accl(rw_buffer_.accl[0], rw_buffer_.accl[1], rw_buffer_.accl[2]);
    Eigen::Vector3f gyro_raw(rw_buffer_.gyro[0], rw_buffer_.gyro[1], rw_buffer_.gyro[2]);

    accl *= GetAccelLSB();
    Eigen::Vector3f gyro = gyro_raw * GetGyroLSB();

    if (in_cali_)
    {
      gyro_cali_.data()[0] += static_cast<int64_t>(rw_buffer_.gyro[0]);
      gyro_cali_.data()[1] += static_cast<int64_t>(rw_buffer_.gyro[1]);
      gyro_cali_.data()[2] += static_cast<int64_t>(rw_buffer_.gyro[2]);
      cali_counter_++;
    }

    gyro -= gyro_offset_key_.data_;

    accl_data_ = accl * rotation_;
    gyro_data_ = gyro * rotation_;

    topic_accl_.Publish(accl_data_);
    topic_gyro_.Publish(gyro_data_);
  }

  static int CommandFunc(QMI8658* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf(
          "  show [time_ms] [interval_ms] - Print sensor data periodically.\r\n");
      LibXR::STDIO::Printf(
          "  list_offset                  - Show current gyro calibration offset.\r\n");
      LibXR::STDIO::Printf(
          "  cali                         - Start gyroscope calibration.\r\n");
    }
    else if (argc == 2)
    {
      if (std::strcmp(argv[1], "list_offset") == 0)
      {
        LibXR::STDIO::Printf("Current calibration offset (dps) - x: %f, y: %f, z: %f\r\n",
                             self->gyro_offset_key_.data_.x(),
                             self->gyro_offset_key_.data_.y(),
                             self->gyro_offset_key_.data_.z());
      }
      else if (std::strcmp(argv[1], "cali") == 0)
      {
        // 第一次：采集 60s 计算偏置
        self->gyro_offset_key_.data_.setZero();
        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0, 0, 0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;

        LibXR::STDIO::Printf(
            "Starting gyroscope calibration. Please keep the device steady.\r\n");
        LibXR::Thread::Sleep(3000);
        for (int i = 0; i < 60; ++i)
        {
          LibXR::STDIO::Printf("Progress: %d / 60\r\n", i + 1);
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

        LibXR::STDIO::Printf("Calibration result (dps) - x: %f, y: %f, z: %f\r\n",
                             self->gyro_offset_key_.data_.x(),
                             self->gyro_offset_key_.data_.y(),
                             self->gyro_offset_key_.data_.z());

        // 第二次：采集 60s 评估误差
        LibXR::STDIO::Printf("Analyzing calibration quality...\r\n");
        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0, 0, 0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;
        for (int i = 0; i < 60; ++i)
        {
          LibXR::STDIO::Printf("Progress: %d / 60\r\n", i + 1);
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

        LibXR::STDIO::Printf("Calibration error (dps) - x: %f, y: %f, z: %f\r\n",
                             mean_x - self->gyro_offset_key_.data_.x(),
                             mean_y - self->gyro_offset_key_.data_.y(),
                             mean_z - self->gyro_offset_key_.data_.z());

        self->gyro_offset_key_.Set(self->gyro_offset_key_.data_);
        LibXR::STDIO::Printf("Calibration data saved.\r\n");
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
          LibXR::STDIO::Printf(
              "Accel: x = %+5f, y = %+5f, z = %+5f | Gyro: x = %+5f, y = %+5f, z = %+5f "
              "| Temp: %+5f\r\n",
              self->accl_data_.x(), self->accl_data_.y(), self->accl_data_.z(),
              self->gyro_data_.x(), self->gyro_data_.y(), self->gyro_data_.z(),
              self->temperature_);
          LibXR::Thread::Sleep(delay);
          time -= delay;
        }
      }
    }
    else
    {
      LibXR::STDIO::Printf("Error: Invalid arguments.\r\n");
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

  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_offset_key_;
};
