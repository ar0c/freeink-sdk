"""Build the real Metalio panel adapter against a recording I2C/GPIO transport."""

from pathlib import Path
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
BOARD = HERE.parents[1]
LCD = BOARD.parents[1] / "display/EpdiyLcd"

STUBS = {
    "Arduino.h": """#pragma once
#include <cstdint>
#include <cstddef>
#include <initializer_list>
#define CONFIG_IDF_TARGET_ESP32S31 1
extern unsigned long clockUs;
inline unsigned long millis() { return clockUs / 1000; }
inline void delay(unsigned long ms) { clockUs += ms * 1000; }
inline void delayMicroseconds(unsigned long us) { clockUs += us; }
""",
    "driver/gpio.h": """#pragma once
#define ESP_OK 0
using gpio_num_t = int;
inline constexpr int GPIO_NUM_18 = 18, GPIO_NUM_19 = 19, GPIO_NUM_37 = 37;
inline constexpr int GPIO_MODE_OUTPUT = 2;
inline int gpio_hold_dis(int) { return ESP_OK; }
inline int gpio_set_direction(int, int) { return ESP_OK; }
int gpio_set_level(int, int);
""",
    "esp_log.h": "#define ESP_LOGE(...) ((void)0)\n",
    "Wire.h": """#pragma once
#include <cstdint>
#include <cstddef>
struct WireFixture {
    uint8_t address;
    uint8_t payload[2];
    size_t count;
    bool begin(int sda, int scl, uint32_t hz);
    void setTimeOut(uint16_t) {}
    void beginTransmission(uint8_t target) { address = target; count = 0; }
    void write(uint8_t byte) { payload[count++] = byte; }
    uint8_t endTransmission(bool stop = true);
    size_t requestFrom(uint8_t target, size_t bytes);
    int read();
};
extern WireFixture Wire1;
""",
}


def run():
    with tempfile.TemporaryDirectory(prefix="freeink-metalio-panel-") as directory:
        root = Path(directory)
        for path, source in STUBS.items():
            output = root / path
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(source)
        binary = root / "panel"
        subprocess.run([
            "c++", "-std=c++20", "-DFREEINK_DEVICE_METALIO_EINK4_PLUS=1",
            "-I" + str(root), "-I" + str(LCD / "include"),
            "-I" + str(LCD / "src/epdiy/include"), "-I" + str(LCD / "src/epdiy/src"),
            str(HERE / "panel.cpp"), str(BOARD / "src/BoardMetalioEInk4Plus.cpp"),
            str(BOARD / "src/FactoryWaveform.cpp"), "-o", str(binary),
        ], check=True)
        for scenario in ("normal", "no-power-good", "wrong-pmic", "vcom-write-fails", "shutdown-fails", "ambiguous"):
            subprocess.run([str(binary), scenario], check=True)


if __name__ == "__main__":
    run()
