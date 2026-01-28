# include "lowpass.h"

void LowPass_Filter(float* input, float* output, float alpha) {
    *output = alpha * (*input) + (1.0f - alpha) * (*output);
}