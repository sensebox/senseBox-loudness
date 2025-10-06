/*
 * ESP32 Second-Order Sections IIR Filter implementation
 *
 * (c)2019 Ivan Kostoski
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *    
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SOS_IIR_FILTER_H
#define SOS_IIR_FILTER_H

#include <stdint.h>
#include <math.h>
#include <string.h>

struct SOS_Coefficients {
  float b1;
  float b2;
  float a1;
  float a2;
};

struct SOS_Delay_State {
  float w0 = 0;
  float w1 = 0;
};

//
// Assembly-optimized implementation for ESP32-S3 only
//
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3)

extern "C" {
  int sos_filter_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w);
} 
__asm__ (
  ".text                    \n"
  ".align  4                \n"
  ".global sos_filter_f32   \n"
  ".type   sos_filter_f32,@function\n"
  "sos_filter_f32:          \n"
  "  entry   a1, 16         \n"
  "  lsi     f0, a5, 0      \n"
  "  lsi     f1, a5, 4      \n"
  "  lsi     f2, a5, 8      \n"
  "  lsi     f3, a5, 12     \n"
  "  lsi     f4, a6, 0      \n"
  "  lsi     f5, a6, 4      \n"
  "  loopnez a4, 1f         \n"
  "    lsip    f6, a2, 4    \n"
  "    madd.s  f6, f2, f4   \n"
  "    madd.s  f6, f3, f5   \n"
  "    mov.s   f7, f6       \n"
  "    madd.s  f7, f0, f4   \n"
  "    madd.s  f7, f1, f5   \n"
  "    ssip    f7, a3, 4    \n"
  "    mov.s   f5, f4       \n"
  "    mov.s   f4, f6       \n"
  "  1:                     \n"
  "  ssi     f4, a6, 0      \n"
  "  ssi     f5, a6, 4      \n"
  "  movi.n   a2, 0         \n"
  "  retw.n                 \n"
);

extern "C" {
  float sos_filter_sum_sqr_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w, float gain);
}
__asm__ (
  ".text                    \n"
  ".align  4                \n"
  ".global sos_filter_sum_sqr_f32 \n"
  ".type   sos_filter_sum_sqr_f32,@function \n"
  "sos_filter_sum_sqr_f32:  \n"
  "  entry   a1, 16         \n" 
  "  lsi     f0, a5, 0      \n"
  "  lsi     f1, a5, 4      \n"
  "  lsi     f2, a5, 8      \n"
  "  lsi     f3, a5, 12     \n"
  "  lsi     f4, a6, 0      \n"
  "  lsi     f5, a6, 4      \n"
  "  wfr     f6, a7         \n"
  "  const.s f10, 0         \n"
  "  loopnez a4, 1f         \n"
  "    lsip    f7, a2, 4    \n"
  "    madd.s  f7, f2, f4   \n"
  "    madd.s  f7, f3, f5   \n"
  "    mov.s   f8, f7       \n"
  "    madd.s  f8, f0, f4   \n"
  "    madd.s  f8, f1, f5   \n"
  "    mul.s   f9, f8, f6   \n"
  "    ssip    f9, a3, 4    \n"
  "    mov.s   f5, f4       \n"
  "    mov.s   f4, f7       \n"
  "    madd.s  f10, f9, f9  \n"
  "  1:                     \n"
  "  ssi     f4, a6, 0      \n"
  "  ssi     f5, a6, 4      \n"
  "  rfr     a2, f10        \n"
  "  retw.n                 \n"
);

//
// Portable C implementation for ESP32-S2 and others
//
#else

extern "C" int sos_filter_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w) {
  for (int i = 0; i < len; i++) {
    float wn = input[i] + coeffs.a1 * w.w0 + coeffs.a2 * w.w1;
    float yn = wn + coeffs.b1 * w.w0 + coeffs.b2 * w.w1;
    output[i] = yn;
    w.w1 = w.w0;
    w.w0 = wn;
  }
  return 0;
}

extern "C" float sos_filter_sum_sqr_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w, float gain) {
  float sum_sqr = 0;
  for (int i = 0; i < len; i++) {
    float wn = input[i] + coeffs.a1 * w.w0 + coeffs.a2 * w.w1;
    float yn = (wn + coeffs.b1 * w.w0 + coeffs.b2 * w.w1) * gain;
    output[i] = yn;
    sum_sqr += yn * yn;
    w.w1 = w.w0;
    w.w0 = wn;
  }
  return sum_sqr;
}

#endif // CONFIG_IDF_TARGET_ESP32S3

//
// C++ wrapper (common for both S2 and S3)
//
struct SOS_IIR_Filter {
  const int num_sos;
  const float gain;
  SOS_Coefficients* sos = NULL;
  SOS_Delay_State* w = NULL;

  SOS_IIR_Filter(size_t num_sos, const float gain, const SOS_Coefficients _sos[] = NULL) 
    : num_sos(num_sos), gain(gain) {
    if (num_sos > 0) {
      sos = new SOS_Coefficients[num_sos];
      if ((sos != NULL) && (_sos != NULL)) memcpy(sos, _sos, num_sos * sizeof(SOS_Coefficients));
      w = new SOS_Delay_State[num_sos]();
    }
  };

  template <size_t Array_Size>
  SOS_IIR_Filter(const float gain, const SOS_Coefficients (&sos)[Array_Size]) 
    : SOS_IIR_Filter(Array_Size, gain, sos) {};

  inline float filter(float* input, float* output, size_t len) {
    if ((num_sos < 1) || (sos == NULL) || (w == NULL)) return 0;
    float* source = input; 
    for(int i = 0; i < (num_sos-1); i++) {
      sos_filter_f32(source, output, len, sos[i], w[i]);
      source = output;
    }
    return sos_filter_sum_sqr_f32(source, output, len, sos[num_sos-1], w[num_sos-1], gain);
  }

  ~SOS_IIR_Filter() {
    if (w != NULL) delete[] w;
    if (sos != NULL) delete[] sos;
  }
};

struct No_IIR_Filter {
  const int num_sos = 0;
  const float gain = 1.0;
  No_IIR_Filter() {};
  inline float filter(float* input, float* output, size_t len) {
    float sum_sqr = 0;
    for(int i=0; i<len; i++) {
      float s = input[i];
      sum_sqr += s * s;
    }
    if (input != output) {
      for(int i=0; i<len; i++) output[i] = input[i];
    }
    return sum_sqr;
  };
};

No_IIR_Filter None;

#endif // SOS_IIR_FILTER_H
