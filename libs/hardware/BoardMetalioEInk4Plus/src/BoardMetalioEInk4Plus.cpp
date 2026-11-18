// SPDX-License-Identifier: Apache-2.0

#include <Arduino.h>
#include <EpdiyLcd.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <esp_log.h>

#if CONFIG_IDF_TARGET_ESP32S31 && FREEINK_DEVICE_METALIO_EINK4_PLUS

namespace freeink {
namespace metalio_plus {
const EpdDisplay& factoryPanel();
}
namespace {

// Pin map and sequencing from CloudZao/Metalio-E-INK4-Plus 2.0.79 (14aa1d06),
// config.h and display/epd_board_metalio_eink4_plus.c. Datasheets: TI
// SCPS200E (TCA9555) and SLVSAQ8G (TPS65185).
constexpr char kTag[] = "MetalioPlusPanel";
constexpr gpio_num_t kOutputEnable = GPIO_NUM_18;
constexpr gpio_num_t kMode = GPIO_NUM_19;
constexpr gpio_num_t kBorder = GPIO_NUM_37;
constexpr uint8_t kPmicAddress = 0x68;
constexpr uint8_t kInput0 = 0x00;
constexpr uint8_t kOutput0 = 0x02;
constexpr uint8_t kOutput1 = 0x03;
constexpr uint8_t kConfig0 = 0x06;
constexpr uint8_t kConfig1 = 0x07;
constexpr uint8_t kVcom = 1 << 0;
constexpr uint8_t kPowerUp = 1 << 1;
constexpr uint8_t kWakeUp = 1 << 2;
constexpr uint8_t kPanelSupply = 1 << 5;
constexpr uint8_t kPowerGoodBit = 1 << 7;
constexpr uint8_t kPanelControl = kVcom | kPowerUp | kWakeUp;
constexpr uint8_t kSharedResetAndSupply = (1 << 2) | (1 << 4);  // P12 and P14.
constexpr uint8_t kPmicEnable = 0x01;
constexpr uint8_t kPmicVcom1 = 0x03;
constexpr uint8_t kPmicVcom2 = 0x04;
constexpr uint8_t kPmicUpseq0 = 0x09;
constexpr uint8_t kPmicUpseq1 = 0x0A;
constexpr uint8_t kPmicPg = 0x0F;
constexpr uint8_t kPmicRevision = 0x10;
constexpr uint8_t kAllRailsGood = 0xFA;
constexpr uint16_t kVcom10mV = 120;  // Stock black-depth default; volatile only.
constexpr uint32_t kWakeDelayMs = 50;
constexpr uint32_t kPowerGoodTimeoutMs = 200;

enum class State { Unprepared, Off, Starting, Active, Fault };
State state = State::Unprepared;
uint8_t expanderAddress = 0;

bool readRegister(uint8_t address, uint8_t reg, uint8_t& value) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  // The factory driver uses STOP for TPS reads. The expander supports repeated START.
  if (Wire1.endTransmission(address == kPmicAddress) != 0 || Wire1.requestFrom(address, size_t{1}) != 1) {
    return false;
  }
  const int byte = Wire1.read();
  if (byte < 0) return false;
  value = static_cast<uint8_t>(byte);
  return true;
}

bool writeVerified(uint8_t address, uint8_t reg, uint8_t value, uint8_t mask = 0xFF) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.write(value);
  uint8_t actual = 0;
  return Wire1.endTransmission() == 0 && readRegister(address, reg, actual) && ((actual ^ value) & mask) == 0;
}

bool updateExpander(uint8_t reg, uint8_t mask, uint8_t selected) {
  uint8_t value;
  if (!readRegister(expanderAddress, reg, value)) return false;
  const uint8_t next = (value & ~mask) | (selected & mask);
  return next == value || writeVerified(expanderAddress, reg, next);
}

bool parkPanelPins() {
  for (const auto pin : {kOutputEnable, kMode, kBorder}) {
    if (gpio_hold_dis(pin) != ESP_OK || gpio_set_level(pin, 0) != ESP_OK ||
        gpio_set_direction(pin, GPIO_MODE_OUTPUT) != ESP_OK)
      return false;
  }
  return true;
}

void powerOff() {
  if (state == State::Unprepared || state == State::Off || state == State::Fault) return;
  const bool outputDisabled = gpio_set_level(kOutputEnable, 0) == ESP_OK;
  const bool modeLow = gpio_set_level(kMode, 0) == ESP_OK;
  // The stock panel waits at least 12 us between XOE low and dropping VPOS/VNEG.
  delayMicroseconds(12);
  bool down = updateExpander(kOutput0, kVcom | kPowerUp, 0);
  if (!down) down = updateExpander(kOutput0, kVcom | kPowerUp, 0);
  delay(2);
  const bool asleep = down && updateExpander(kOutput0, kWakeUp, 0);
  if (!outputDisabled || !modeLow || !asleep) {
    state = State::Fault;
    ESP_LOGE(kTag, "Panel power-down could not be verified");
    return;
  }
  state = State::Off;
}

bool preparePanel() {
  if (state != State::Unprepared) return state == State::Off;
  if (!parkPanelPins() || !Wire1.begin(0, 1, 400000)) return false;
  Wire1.setTimeOut(20);
  for (uint8_t addr = 0x20; addr <= 0x27; ++addr) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() != 0) continue;
    if (expanderAddress != 0) {
      state = State::Fault;
      ESP_LOGE(kTag, "Ambiguous panel expander");
      return false;
    }
    expanderAddress = addr;
  }
  if (expanderAddress == 0) {
    state = State::Fault;
    ESP_LOGE(kTag, "No panel expander");
    return false;
  }
  // Set safe latch values before switching the expander pins to output.
  const bool configured = updateExpander(kOutput0, kPanelControl | kPanelSupply, kPanelSupply) &&
                          updateExpander(kConfig0, kPanelControl | kPanelSupply | kPowerGoodBit, kPowerGoodBit) &&
                          updateExpander(kOutput1, kSharedResetAndSupply, kSharedResetAndSupply) &&
                          updateExpander(kConfig1, kSharedResetAndSupply, 0);
  if (!configured) {
    state = State::Fault;
    ESP_LOGE(kTag, "Panel expander preparation failed");
    return false;
  }
  delay(80);  // Factory POWER_RAIL_SETTLE_MS.
  state = State::Off;
  return true;
}

bool waitRegister(uint8_t address, uint8_t reg, uint8_t mask, uint8_t expected) {
  const auto started = millis();
  do {
    uint8_t value;
    if (!readRegister(address, reg, value)) return false;
    if ((value & mask) == expected) return true;
    delay(2);
  } while (millis() - started < kPowerGoodTimeoutMs);
  return false;
}

bool startRails() {
  if (!updateExpander(kOutput0, kPanelControl, kWakeUp)) return false;
  delay(kWakeDelayMs);
  uint8_t revision;
  if (!readRegister(kPmicAddress, kPmicRevision, revision) ||
      (revision != 0x45 && revision != 0x55 && revision != 0x65 && revision != 0x66))
    return false;
  if (!writeVerified(kPmicAddress, kPmicUpseq0, 0xE4) || !writeVerified(kPmicAddress, kPmicUpseq1, 0x55)) return false;
  // Match the factory PWRUP/VCOM_CTRL edge, then check external PWR_GOOD
  // before enabling rails and writing the operating voltage registers.
  if (!updateExpander(kOutput0, kPowerUp | kVcom, kPowerUp | kVcom)) return false;
  delay(50);
  if (!waitRegister(expanderAddress, kInput0, kPowerGoodBit, kPowerGoodBit)) return false;
  if (!writeVerified(kPmicAddress, kPmicEnable, 0x3F, 0x3F)) return false;
  // VCOM2 contains the nonvolatile PROG bit and the acquisition control.
  // Write only bit 0, the high bit of the volatile DAC value.
  if (!writeVerified(kPmicAddress, kPmicVcom2, (kVcom10mV >> 8) & 1, 0xE1) ||
      !writeVerified(kPmicAddress, kPmicVcom1, kVcom10mV & 0xFF))
    return false;
  return waitRegister(kPmicAddress, kPmicPg, kAllRailsGood, kAllRailsGood) &&
         gpio_set_level(kOutputEnable, 1) == ESP_OK;
}

bool powerOn() {
  if (state == State::Active) return true;
  if (state != State::Off) return false;
  state = State::Starting;
  if (!startRails()) {
    ESP_LOGE(kTag, "Panel power-on failed");
    powerOff();
    return false;
  }
  state = State::Active;
  return true;
}

void setPanelMode(bool enabled) {
  const bool scanning = enabled && state == State::Active;
  gpio_set_level(kOutputEnable, scanning);
  gpio_set_level(kMode, scanning);
}

}  // namespace

const EpdiyLcdConfig& metalioEink4PlusEpdiyConfig() {
  static const EpdiyLcdConfig config{
      .dataPins = {8, 9, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1},
      .pinClock = 16,
      .pinCkv = 40,
      .pinStartPulse = 4,
      .pinLeh = 17,
      .pinStv = 35,
      .busWidth = 8,
      .pclkMhz = 20,
      .line = {9, 4, 10, 60},
      .prefillLines = 126,
      .power = {preparePanel, powerOn, powerOff, nullptr, setPanelMode},
      .display = &metalio_plus::factoryPanel(),
      .minPhaseIntervalUs = 11765,  // Stock phase rate is 85 Hz.
  };
  return config;
}

}  // namespace freeink

#endif
