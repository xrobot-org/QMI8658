# QMI8658

**模块描述 / Module Description**  
上海矽睿科技有限公司 QMI8658 6 轴惯性测量单元（IMU）的驱动模块。  
Driver module for the QMI8658 6-axis Inertial Measurement Unit (IMU) from QST Corporation Limited.

---

## 硬件需求 / Required Hardware
- `spi_name`
- `cs_pin_name`
- `int_pin1_name`
- `int_pin2_name`
- `pwm_name`
- `ramfs`
- `database`

---

## 构造参数 / Constructor Arguments

- `output_freq`: `QMI8658::ODR::ODR_896_8HZ`  
- `gyro_range`: `QMI8658::GyroRange::DEG_2048DPS`  
- `accl_range`: `QMI8658::AcclRange::ACCL_16G`  
- `accl_lpf`: `QMI8658::ModeLPF::LFP_DISABLE`  
- `gyro_lpf`: `QMI8658::ModeLPF::LFP_DISABLE`  
- `rotation`: `{ w: 1.0, x: 0.0, y: 0.0, z: 0.0 }`  
- `pid_param`: `{ k: 1.0, p: 0.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 0.0, cycle: false }`  
- `gyro_topic_name`: `"qmi8658_gyro"`  
- `accl_topic_name`: `"qmi8658_accl"`  
- `spi_name`: `"spi1"`  
- `cs_pin_name`: `"spi1_cs"`  
- `int_pin2_name`: `"spi1_int2"`  
- `pwm_name`: `"pwm1"`  
- `target_temperature`: `45`  

---

## 依赖 / Depends
[]