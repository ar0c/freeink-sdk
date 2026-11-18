"""Compile the production S31 pulse packing and crop mask on the host."""

from pathlib import Path
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
EPDIY = HERE.parents[1] / "src/epdiy"
COMMON = EPDIY / "src/output_common"

STUBS = {
    "sdkconfig.h": "#define CONFIG_IDF_TARGET_ESP32S31 1\n",
    "esp_attr.h": "#define IRAM_ATTR\n",
    "esp_timer.h": "",
    "esp_err.h": "typedef int esp_err_t;\n",
    "esp_log.h": "",
    "driver/gpio.h": "typedef int gpio_num_t;\n",
    "freertos/FreeRTOS.h": "#include <assert.h>\n",
    "freertos/task.h": "typedef void* TaskHandle_t;\n",
    "freertos/semphr.h": "typedef void* SemaphoreHandle_t;\n",
}


def run():
    with tempfile.TemporaryDirectory(prefix="freeink-s31-packing-") as directory:
        root = Path(directory)
        for name, text in STUBS.items():
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(text)
        executable = root / "s31-packing"
        subprocess.run([
            "cc", "-std=gnu11", "-O2", "-Werror=implicit-function-declaration",
            "-I" + str(root), "-I" + str(EPDIY / "include"), "-I" + str(EPDIY / "src"),
            "-I" + str(COMMON), str(HERE / "s31_packing.c"), str(COMMON / "lut.c"),
            str(COMMON / "render_context.c"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    run()
