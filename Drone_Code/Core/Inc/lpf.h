#ifndef LPF_H
#define LPF_H

#include <stdint.h>

typedef struct {
    float cutoff_freq;
    float dt;
    float output;
    uint8_t initialized;
} LPF_t;

void lpf_init(LPF_t *lpf, float cutoff_freq, float dt);
float lpf_update(LPF_t *lpf, float input);

#endif