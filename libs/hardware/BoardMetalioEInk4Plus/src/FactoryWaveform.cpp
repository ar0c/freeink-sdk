// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Shenzhen Xinzhi Future Technology Co., Ltd.
// Copyright (c) 2025 Project Contributors
// Balanced GC/GL and completed DU adapted from CloudZao/Metalio-E-INK4-Plus
// 14aa1d06, display/waveforms/my_waveform.c. See LICENSE.waveforms.

#include <array>
#include <cstddef>
#include <cstdint>

#include "epd_display.h"
#include "epd_waveform.h"

namespace freeink::metalio_plus {
namespace {

constexpr size_t kBytesPerPhase = 64;
template <size_t Phases>
using WaveformTable = std::array<uint8_t, Phases * kBytesPerPhase>;

#include "FactoryWaveformData.inc"

constexpr size_t index(size_t phase, unsigned to, unsigned from) { return phase * kBytesPerPhase + to * 4 + from / 4; }

constexpr unsigned shift(unsigned from) { return 6 - 2 * (from % 4); }

template <size_t Bytes>
constexpr unsigned drive(const std::array<uint8_t, Bytes>& table, size_t phase, unsigned to, unsigned from) {
  return (table[index(phase, to, from)] >> shift(from)) & 3;
}

constexpr auto completeDu() {
  WaveformTable<20> table{};
  for (size_t phase = 0; phase < 20; ++phase) {
    for (unsigned to = 0; to < 16; ++to) {
      for (unsigned from = 0; from < 16; ++from) {
        table[index(phase, to, from)] |= drive(kStockDU, phase, to < 8 ? 0 : 15, from) << shift(from);
      }
    }
  }
  for (size_t phase = 16; phase < 20; ++phase) table[index(phase, 0, 0)] |= 1 << shift(0);
  return table;
}

constexpr auto maintainWhiteGl() {
  auto table = kStockGL;
  size_t whiteTick = 45;
  for (size_t phase = 48; phase > 0; --phase) {
    bool drivesWhite = false;
    for (unsigned from = 0; from < 15; ++from) drivesWhite |= drive(table, phase - 1, 15, from) == 2;
    if (drivesWhite) {
      whiteTick = phase - 1;
      break;
    }
  }
  table[index(whiteTick, 15, 15)] |= 2 << shift(15);
  return table;
}

constexpr auto kDu = completeDu();
constexpr auto kGl = maintainWhiteGl();
static_assert(drive(kDu, 19, 0, 0) == 1);
static_assert(kStockGC.size() == 48 * kBytesPerPhase);

const EpdWaveformPhases kDuPhases{20, kDu.data(), nullptr};
const EpdWaveformPhases kGcPhases{48, kStockGC.data(), nullptr};
const EpdWaveformPhases kGlPhases{48, kGl.data(), nullptr};
const EpdWaveformPhases* kDuRanges[]{&kDuPhases};
const EpdWaveformPhases* kGcRanges[]{&kGcPhases};
const EpdWaveformPhases* kGlRanges[]{&kGlPhases};
const EpdWaveformMode kDuMode{1, 1, kDuRanges};
const EpdWaveformMode kGcMode{2, 1, kGcRanges};
const EpdWaveformMode kGlMode{5, 1, kGlRanges};
const EpdWaveformMode* kModes[]{&kDuMode, &kGcMode, &kGlMode};
constexpr EpdWaveformTempInterval kBalancedTemperature{0, 50};
const EpdWaveform kFactoryBalanced{3, 1, kModes, &kBalancedTemperature};

}  // namespace

const EpdDisplay& factoryPanel() {
  static constexpr EpdDisplay panel{1216, 684, 8, 20, &kFactoryBalanced};
  return panel;
}

}  // namespace freeink::metalio_plus
