#include <assert.h>
#include <stdalign.h>
#include <stdint.h>
#include <string.h>

#include "lut.h"
#include "render_context.h"

int main(void) {
    // Hand-packed phases: transitions with 'to' shades 0, 1, 2, 3 produce
    // drives 0, 1, 2, 3 respectively. Old shade is irrelevant for this fixture.
    uint8_t packed[128] = {0};
    for (int to = 0; to < 16; to++) memset(packed + 64 + to * 4, (to % 4) * 0x55, 4);
    EpdWaveformPhases phase = {2, packed, NULL};
    alignas(16) uint8_t lut[65536];
    alignas(16) uint8_t input[16] = {0x03, 0x12, 0x21, 0x30, 0x37, 0x2A, 0x1E, 0x08};
    alignas(16) uint8_t out[4] = {0, 0, 0, 0xA5};

    LutFunctionPair pair = find_lut_functions(MODE_PACKING_1PPB_DIFFERENCE, 1024);
    assert(pair.build_func && pair.lookup_func);
    pair.build_func(lut, &phase, 1);
    pair.lookup_func((const uint32_t*)input, out, lut, 8);
    assert(out[0] == 0x1B && out[1] == 0xE4 && out[3] == 0xA5);

    for (int size = 1024; size <= 65536; size *= 64) {
        pair = find_lut_functions(MODE_PACKING_2PPB | PREVIOUSLY_WHITE, size);
        assert(pair.build_func && pair.lookup_func);
        input[0] = 0x10;
        input[1] = 0x32;
        pair.build_func(lut, &phase, 1);
        pair.lookup_func((const uint32_t*)input, out, lut, 4);
        assert(out[0] == 0x1B);
    }

    pair = find_lut_functions(MODE_PACKING_8PPB | PREVIOUSLY_WHITE, 1024);
    assert(pair.build_func && pair.lookup_func);
    input[0] = 0xFD;  // Only pixel 1 darkens; native encoding puts drive 1 in bits 5:4.
    pair.build_func(lut, &phase, 0);
    pair.lookup_func((const uint32_t*)input, out, lut, 4);
    assert(out[0] == 0x10);

    alignas(4) uint8_t mask[4];
    const uint8_t columns[] = {0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF};
    epd_populate_line_mask(mask, columns, sizeof(mask));
    assert(mask[0] == 0xC0 && mask[1] == 0x03 && mask[2] == 0 && mask[3] == 0xFF);
    memset(out, 0x55, sizeof(out));
    epd_apply_line_mask(out, mask, sizeof(out));
    assert(out[0] == 0x40 && out[1] == 0x01 && out[2] == 0 && out[3] == 0x55);
    return 0;
}
