#include "lpf.h"
#include <math.h>

void lpf_init(LPF_t *lpf, float cutoff_freq, float dt) {
    lpf->cutoff_freq = cutoff_freq;
    lpf->dt = dt;
    lpf->output = 0.0f;
    lpf->initialized = 0;
}

float lpf_update(LPF_t *lpf, float input) {
    if (!lpf->initialized) {
        lpf->output = input;
        lpf->initialized = 1;
        return lpf->output;
    }

    // Calculate smoothing factor alpha based on cutoff frequency and dt
    // RC filter formula: alpha = dt / (RC + dt), where RC = 1 / (2 * pi * fc)
    float rc = 1.0f / (2.0f * 3.14159265f * lpf->cutoff_freq);
    float alpha = lpf->dt / (rc + lpf->dt);

    // Clamp alpha between 0.0 and 1.0 for safety
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    lpf->output = lpf->output + alpha * (input - lpf->output);
    return lpf->output;
}