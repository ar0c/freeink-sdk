#include <Arduino.h>
#include <EpdiyLcd.h>
#include <Wire.h>

#include <algorithm>
#include <cassert>
#include <string_view>
#include <vector>

#include "epd_display.h"

namespace freeink {
const EpdiyLcdConfig& metalioEink4PlusEpdiyConfig();
}

namespace {
constexpr uint8_t kTca = 0x22, kTps = 0x68;
constexpr uint8_t kGateMask = 7, kWake = 4;
std::string_view scenario;
uint8_t tca[8]{};
uint8_t tps[17]{};
int pin18 = 0;
bool refuseOff = false;
unsigned long lastXoeFell = 0;
struct Write {
  uint8_t chip;
  uint8_t reg;
  uint8_t value;
  unsigned long time;
};
std::vector<Write> writes;

size_t findWrite(uint8_t chip, uint8_t reg, uint8_t value) {
  const auto match = std::find_if(writes.begin(), writes.end(),
                                  [=](const Write& w) { return w.chip == chip && w.reg == reg && w.value == value; });
  assert(match != writes.end());
  return match - writes.begin();
}
}  // namespace

unsigned long clockUs = 0;
WireFixture Wire1;

int gpio_set_level(int pin, int level) {
  if (pin == 18) {
    if (pin18 == 1 && level == 0) lastXoeFell = clockUs;
    if (level == 1) {
      assert((tca[2] & kGateMask) == kGateMask);
      assert(tps[3] == 120 && tps[4] == 0);
    }
    pin18 = level;
  }
  return 0;
}

bool WireFixture::begin(int sda, int scl, uint32_t hz) { return sda == 0 && scl == 1 && hz == 400000; }

uint8_t WireFixture::endTransmission(bool stop) {
  if (address != kTca && address != kTps) {
    return scenario == "ambiguous" && address == 0x25 ? 0 : 2;
  }
  if (count == 1) assert(stop == (address == kTps));
  if (count != 2) return 0;
  const uint8_t reg = payload[0], value = payload[1];
  if (address == kTps && reg == 4) {
    // Only the volatile DAC's high bit may be written.
    assert((value & ~1) == 0);
  }
  if (address == kTps && reg == 3 && scenario == "vcom-write-fails") return 4;
  if (address == kTca && reg == 2 && (tca[2] & 3) != 0 && (value & 3) == 0) {
    if (refuseOff) return 4;
    assert(pin18 == 0);
    if (lastXoeFell > 0) assert(clockUs >= lastXoeFell + 12);
  }
  (address == kTca ? tca : tps)[reg] = value;
  writes.push_back({address, reg, value, clockUs});
  return 0;
}

size_t WireFixture::requestFrom(uint8_t target, size_t bytes) {
  assert(target == address && bytes == 1);
  return bytes;
}

int WireFixture::read() {
  if (address == kTca && payload[0] == 0) {
    return (tca[2] & 2) && scenario != "no-power-good" ? 0x80 : 0;
  }
  return (address == kTca ? tca : tps)[payload[0]];
}

int main(int argc, char** argv) {
  assert(argc == 2);
  scenario = argv[1];
  tca[6] = tca[7] = 0xFF;
  tps[15] = 0xFA;
  tps[16] = scenario == "wrong-pmic" ? 0xFF : 0x45;

  const auto& board = freeink::metalioEink4PlusEpdiyConfig();
  assert(board.display->width == 1216 && board.display->height == 684);
  assert(board.display->bus_width == 8 && board.display->bus_speed == 20);
  const auto& wave = *board.display->default_waveform;
  assert(wave.num_modes == 3 && wave.num_temp_ranges == 1);
  assert(wave.mode_data[0]->range_data[0]->phases == 20);
  assert(wave.mode_data[1]->range_data[0]->phases == 48);
  assert(wave.mode_data[2]->range_data[0]->phases == 48);
  if (scenario == "ambiguous") {
    assert(!board.power.prepare() && writes.empty());
    return 0;
  }

  assert(board.power.prepare());
  assert((tca[2] & kGateMask) == 0 && pin18 == 0);
  if (scenario != "normal" && scenario != "shutdown-fails") {
    assert(!board.power.powerOn());
    assert((tca[2] & kGateMask) == 0 && pin18 == 0);
    return 0;
  }

  assert(board.power.powerOn() && pin18 == 1);
  const size_t wake = findWrite(kTca, 2, 0x20 | kWake);
  const size_t upseq = findWrite(kTps, 9, 0xE4);
  const size_t secondUpseq = findWrite(kTps, 10, 0x55);
  const size_t active = findWrite(kTca, 2, 0x20 | kGateMask);
  const size_t enable = findWrite(kTps, 1, 0x3F);
  assert(wake < upseq && upseq < secondUpseq && secondUpseq < active && active < enable);
  assert(writes[upseq].time - writes[wake].time >= 50000);
  assert(writes[enable].time - writes[active].time >= 50000);

  refuseOff = scenario == "shutdown-fails";
  board.power.powerOff();
  assert(pin18 == 0);
  if (refuseOff) {
    const size_t prior = writes.size();
    assert(!board.power.powerOn() && writes.size() == prior);
  } else {
    assert((tca[2] & kGateMask) == 0);
  }
}
