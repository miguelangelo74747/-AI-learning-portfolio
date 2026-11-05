// =============================================================================
//  mpu6050.cpp — ver mpu6050.h
// =============================================================================
#include "mpu6050.h"

#include "config.h"

namespace mpu {

bool Mpu6050::writeReg(uint8_t reg, uint8_t val) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(val);
  return wire_.endTransmission() == 0;
}

bool Mpu6050::readRegs(uint8_t reg, uint8_t* buf, size_t n) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;  // repeated start
  if (wire_.requestFrom(addr_, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; ++i) {
    const int b = wire_.read();
    if (b < 0) return false;
    buf[i] = (uint8_t)b;
  }
  return true;
}

InitResult Mpu6050::begin() {
  uint8_t v = 0;
  who_am_i_ = 0;

  // ¿Responde alguien en 0x68?
  if (!readRegs(REG_WHO_AM_I, &v, 1)) return InitResult::NO_ACK;
  who_am_i_ = v;
  if (v != MPU_WHO_AM_I_EXPECTED) return InitResult::BAD_WHO_AM_I;

  // Reset de registros para partir de un estado conocido; el datasheet pide
  // esperar ~100 ms antes de volver a escribir.
  if (!writeReg(REG_PWR_MGMT_1, PWR_DEVICE_RESET)) return InitResult::NO_ACK;
  delay(100);
  if (!writeReg(REG_PWR_MGMT_1, PWR_CLK_PLL_XGYRO)) return InitResult::NO_ACK;
  delay(10);

  const uint8_t want_smplrt = MPU_SMPLRT_DIV;
  const uint8_t want_config = MPU_DLPF_CFG & 0x07;
  const uint8_t want_gyro = (uint8_t)((MPU_GYRO_FS_SEL & 0x03) << 3);
  const uint8_t want_accel = (uint8_t)((MPU_ACCEL_AFS_SEL & 0x03) << 3);

  if (!writeReg(REG_SMPLRT_DIV, want_smplrt) || !writeReg(REG_CONFIG, want_config) ||
      !writeReg(REG_GYRO_CONFIG, want_gyro) || !writeReg(REG_ACCEL_CONFIG, want_accel) ||
      !writeReg(REG_INT_ENABLE, INT_DATA_RDY)) {
    return InitResult::NO_ACK;
  }

  // Verificación: leer de vuelta 0x19–0x1C, INT_ENABLE y PWR_MGMT_1.
  uint8_t cfg[4];
  uint8_t int_en = 0, pwr = 0;
  if (!readRegs(REG_SMPLRT_DIV, cfg, 4) || !readRegs(REG_INT_ENABLE, &int_en, 1) ||
      !readRegs(REG_PWR_MGMT_1, &pwr, 1)) {
    return InitResult::NO_ACK;
  }
  if (cfg[0] != want_smplrt || (cfg[1] & 0x07) != want_config || (cfg[2] & 0x18) != want_gyro ||
      (cfg[3] & 0x18) != want_accel || int_en != INT_DATA_RDY || pwr != PWR_CLK_PLL_XGYRO) {
    return InitResult::BAD_CONFIG;
  }

  // Borra un DATA_RDY antiguo y deja asentar el giróscopo (arranque ≈ 30 ms).
  readRegs(REG_INT_STATUS, &v, 1);
  delay(40);
  return InitResult::OK;
}

bool Mpu6050::read(Reading& r) {
  uint8_t b[15];
  if (!readRegs(REG_INT_STATUS, b, sizeof(b))) return false;
  r.data_ready = (b[0] & INT_DATA_RDY) != 0;
  r.ax = (int16_t)((b[1] << 8) | b[2]);
  r.ay = (int16_t)((b[3] << 8) | b[4]);
  r.az = (int16_t)((b[5] << 8) | b[6]);
  // b[7], b[8]: temperatura (no se usa)
  r.gx = (int16_t)((b[9] << 8) | b[10]);
  r.gy = (int16_t)((b[11] << 8) | b[12]);
  r.gz = (int16_t)((b[13] << 8) | b[14]);
  return true;
}

}  // namespace mpu
