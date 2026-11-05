// =============================================================================
//  mpu6050.h — driver mínimo del MPU6050 (GY-521) sobre Wire.
//
//  Registros según "MPU-6000/MPU-6050 Register Map and Descriptions" rev. 4.2.
//  Una lectura = un único burst de 15 bytes desde INT_STATUS (0x3A) hasta
//  GYRO_ZOUT_L (0x48): bandera DATA_RDY + acelerómetro + temperatura + giróscopo.
// =============================================================================
#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace mpu {

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_INT_ENABLE = 0x38;
constexpr uint8_t REG_INT_STATUS = 0x3A;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr uint8_t PWR_DEVICE_RESET = 0x80;
constexpr uint8_t PWR_CLK_PLL_XGYRO = 0x01;  // despierto, reloj PLL del giróscopo X
constexpr uint8_t INT_DATA_RDY = 0x01;

enum class InitResult : uint8_t { OK = 0, NO_ACK, BAD_WHO_AM_I, BAD_CONFIG };

struct Reading {
  bool data_ready;  // bit DATA_RDY_INT de INT_STATUS (se borra al leerlo)
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

class Mpu6050 {
 public:
  explicit Mpu6050(TwoWire& wire, uint8_t addr) : wire_(wire), addr_(addr) {}

  // Arranque completo: reset, despertar, WHO_AM_I y configuración verificada
  // leyéndola de vuelta. Tras perder la alimentación el MPU6050 arranca en
  // modo sleep (W05), por eso el reintento repite toda la secuencia.
  InitResult begin();
  uint8_t lastWhoAmI() const { return who_am_i_; }

  // true si el burst I2C se completó; los datos solo son nuevos si data_ready.
  bool read(Reading& r);

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t* buf, size_t n);

  TwoWire& wire_;
  uint8_t addr_;
  uint8_t who_am_i_ = 0;
};

}  // namespace mpu
