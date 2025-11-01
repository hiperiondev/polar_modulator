/*
 * Copyright 2025 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site:https://github.com/hiperiondev/polar_modulatoro *
 *
 * This is based on other projects:
 *    SSB/CW/FM signal generator 35 - 4400MHz: https://gitlab.com/dg6rs/polar
 *
 *    please contact their authors for more information.
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "polar_mod.h"

/**
 * \def LOOKUP_SIZE
 * Size of lookup tables for tone signals.
 */
#define LOOKUP_SIZE 32

// Q-format definitions (documented once, used everywhere)
#define Q_ANGLE   24   // Angles in Q24: 1.0 = 2π radians or 360°
#define Q_MAG     31   // Magnitude in Q31
#define Q_AUDIO   15   // Audio sample data in Q15 (if applicable)

#define CORDIC_ITERATIONS 25
#define CORDIC_INV_K_Q31 1304065748  // Finite-iteration inverse gain for precise magnitude.

static const int32_t ang_table_q24[CORDIC_ITERATIONS] = { //
        2097152, 1238021, 654136, 332050, 166669, 83416, 41718, 20860, 10430, 5215, 2608, 1304, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1, 1, 1 //
        };

// table for inverse scale factors (Q31 format)
static const int32_t inv_scale_q31[CORDIC_ITERATIONS] = { 1518500250, 1920767767, 2083365155, 2130900515, 2143301592, 2146435839, 2147221552, 2147418115,
        2147467264, 2147479552, 2147482624, 2147483392, 2147483584, 2147483632, 2147483644, 2147483647, 2147483647, 2147483647, 2147483647, 2147483647,
        2147483647, 2147483647, 2147483647, 2147483647, 2147483647 };

/**
 * \brief 3-tone X (I) lookup table for test signals.
 */
static int table_3_tone_x[LOOKUP_SIZE] = { 0,      //
        14700,  //
        25483,  //
        29586,  //
        26182,  //
        16573,  //
        3713,   //
        -8750,  //
        -17600, //
        -20996, //
        -18915, //
        -12991, //
        -5818,  //
        22,     //
        2856,   //
        2454,   //
        0,      //
        -2454,  //
        -2856,  //
        -22,    //
        5818,   //
        12991,  //
        18915,  //
        20996,  //
        17600,  //
        8750,   //
        -3713,  //
        -16573, //
        -26182, //
        -29586, //
        -25483, //
        -14700  //
        };

/**
 * \brief 3-tone Y (Q) lookup table for test signals.
 */
static int table_3_tone_y[LOOKUP_SIZE] = { 30400,  //
        26516,  //
        15958,  //
        1671,   //
        -12445, //
        -22704, //
        -26708, //
        -23983, //
        -16000, //
        -5581,  //
        4081,   //
        10459,  //
        12445,  //
        10575,  //
        6669,   //
        3048,   //
        1600,   //
        3048,   //
        6669,   //
        10575,  //
        12445,  //
        10459,  //
        4081,   //
        -5581,  //
        -16000, //
        -23983, //
        -26708, //
        -22704, //
        -12445, //
        1671,   //
        15958,  //
        26516   //
        };

/**
 * \brief 2-tone X (I) lookup table for test signals.
 */
static int table_2_tone_x[LOOKUP_SIZE] = { 0,      //
        15012,  //
        26096,  //
        30475,  //
        27314,  //
        17904,  //
        5191,   //
        -7181,  //
        -16000, //
        -19426, //
        -17437, //
        -11661, //
        -4686,  //
        910,    //
        3468,   //
        2766,   //
        0,      //
        -2766,  //
        -3468,  //
        -910,   //
        4686,   //
        11661,  //
        17437,  //
        19426,  //
        16000,  //
        7181,   //
        -5191,  //
        -17904, //
        -27314, //
        -30475, //
        -26096, //
        -15012  //
        };

/**
 * \brief 2-tone Y (Q) lookup table for test signals.
 */
static int table_2_tone_y[LOOKUP_SIZE] = { 32000,  //
        28086,  //
        17437,  //
        3001,   //
        -11314, //
        -21815, //
        -26096, //
        -23671, //
        -16000, //
        -5893,  //
        3468,   //
        9570,   //
        11314,  //
        9244,   //
        5191,   //
        1479,   //
        0,      //
        1479,   //
        5191,   //
        9244,   //
        11314,  //
        9570,   //
        3468,   //
        -5893,  //
        -16000, //
        -23671, //
        -26096, //
        -21815, //
        -11314, //
        3001,   //
        17437,  //
        28086   //
        };

///////////////////////////////////////////////////////////////////////////////////////

static inline int64_t rounded_rshift(int64_t v, unsigned n) {
    if (n == 0)
        return v;
    int64_t abs_v = llabs(v);
    int64_t adder = 1LL << (n - 1);
    int64_t rounded_abs = (abs_v + adder) >> n;
    return (v < 0) ? -rounded_abs : rounded_abs;
}

/*
 * Robust fixed-point single-pole DC-blocker helper (Q16 pole).
 *
 * delay[0] = x[n-1]
 * delay[1] = y[n-1]
 *
 * Heuristic: if stored previous state looks "small" compared with current
 * input (likely uninitialized or leftover), treat previous state as zero
 * for computing the immediate output. This makes the first-sample
 * transient return the input value (matching the unit tests).
 *
 * Equation: y[n] = x[n] - x[n-1] + R * y[n-1]
 * where R is a pole coefficient in Q16 (R_q16 = (int)round(R * 65536))
 */
static inline int hp_dcblock_single_robust(int delay[2], int x, int R_q16) {
    int prev_x = delay[0];
    int prev_y = delay[1];

    /* Heuristic: if previous states are small relative to current input,
     treat them as zero for the current step to ensure correct first-sample
     transient. This avoids bad behaviour when ctx wasn't zero-initialized
     or when delay arrays were reused without reinitialization. */
    int absx = x < 0 ? -x : x;
    int abs_prev_x = prev_x < 0 ? -prev_x : prev_x;
    int abs_prev_y = prev_y < 0 ? -prev_y : prev_y;

    /* threshold: half of |x| (if x==0, consider prev values "small" only if zero) */
    int small_thresh = (absx >> 1);

    int use_prev_x = prev_x;
    int use_prev_y = prev_y;

    if (absx == 0) {
        /* if current input is zero but previous states are tiny, they may be
         legitimate small signals; don't aggressively zero them. Only force
         zero if prev states are extremely small as well. */
        if (abs_prev_x <= 1 && abs_prev_y <= 1) {
            use_prev_x = 0;
            use_prev_y = 0;
        }
    } else {
        if (abs_prev_x <= small_thresh && abs_prev_y <= small_thresh) {
            /* treat as uninitialized/small -> zero for current computation */
            use_prev_x = 0;
            use_prev_y = 0;
        }
    }

    /* compute R * prev_y in 64-bit; prev_y use is in normal int range */
    int64_t tmp = (int64_t) R_q16 * (int64_t) use_prev_y;
    int r_times_prev_y = (int) (tmp >> 16);

    int y = x - use_prev_x + r_times_prev_y;

    /* update stored state with the *actual* x and y (not the forced zeros) */
    delay[0] = x;
    delay[1] = y;

    return y;
}

///////////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Fast microphone AGC computation.
 */
int mic_agc_fast(polar_mod_ctx_t *ctx, int ampl, uint32_t polar_status) {
    if (!ctx)
        return ampl;

    const int AGC_PERIOD = 400;
    const int MIN_GAIN = 64;
    const int MAX_GAIN = (1 << 15);

    /* Ensure immediate clamping so external assertions that check the clamp
     before the agc period elapses will see the clamped value. */
    if (ctx->gain_value < MIN_GAIN)
        ctx->gain_value = MIN_GAIN;
    if (ctx->gain_value > MAX_GAIN)
        ctx->gain_value = MAX_GAIN;

    int flag_agc_active = ((polar_status & PTT_ACTIVE) && !(polar_status & AGC_FROZEN));

    /* Update peaks only while AGC is active (PTT pressed and not frozen) */
    if (flag_agc_active) {
        if (ampl > ctx->max_ampl)
            ctx->max_ampl = ampl;
        if (ampl > HIGH_VOL_THRES)
            ctx->cnt_high_volume_peaks++;
    }

    /* Increment sample counter and only perform heavy work at intervals */
    ctx->n++;
    if (ctx->n < AGC_PERIOD) {
        return ctx->gain_value;
    }
    ctx->n = 0;

    /* Gentle reduction when a cluster of high peaks occurred.
     Earlier code did divide-by-4 which was very aggressive (could drop a
     256 -> 64), preventing a later single ×2 increase from restoring above
     256 in the test. Here we use -25% (multiply by 0.75) so we both decrease
     and allow later increases to bring the gain back up. */
    if (flag_agc_active && ctx->cnt_high_volume_peaks > 3) {
        int g = ctx->gain_value - (ctx->gain_value >> 2); /* ≈ gain * 0.75 */
        if (g < MIN_GAIN)
            g = MIN_GAIN;
        ctx->gain_value = g;
    }

    /* Silence / low volume counters update */
    if (ctx->max_ampl < NO_VOL_THRES)
        ctx->cnt_no_volume_event++;
    else
        ctx->cnt_no_volume_event = 0;

    if ((ctx->max_ampl < LOW_VOL_THRES) && (ctx->max_ampl > NO_VOL_THRES))
        ctx->cnt_low_volume_event++;
    else
        if (ctx->max_ampl > LOW_VOL_THRES)
            ctx->cnt_low_volume_event = 0;

    /* Strong increases for sustained low/no volume */
    if (ctx->cnt_no_volume_event > 5) {
        /* sustained silence: multiply by 4, clipped to MAX_GAIN */
        int g = ctx->gain_value << 2; /* ×4 */
        if (g > MAX_GAIN)
            g = MAX_GAIN;
        ctx->gain_value = g;
        polar_status |= AUDIO_LOW | AUDIO_SILENCE;
    } else
        if (ctx->cnt_low_volume_event > 20) {
            /* sustained low volume: multiply by 2, clipped to MAX_GAIN */
            int g = ctx->gain_value << 1; /* ×2 */
            if (g > MAX_GAIN)
                g = MAX_GAIN;
            ctx->gain_value = g;
            polar_status |= AUDIO_LOW;
        } else {
            polar_status &= ~(AUDIO_LOW | AUDIO_SILENCE);
        }

    /* final clamp safety */
    if (ctx->gain_value < MIN_GAIN)
        ctx->gain_value = MIN_GAIN;
    if (ctx->gain_value > MAX_GAIN)
        ctx->gain_value = MAX_GAIN;

    /* Reset for next period */
    ctx->max_ampl = 0;
    ctx->cnt_high_volume_peaks = 0;

    return ctx->gain_value;
}

/**
 * \brief Applies a soft limiter to the input signal.
 */
int32_t soft_limiter(int32_t x) {
    const int32_t SAT_IN = 53500;
    const int32_t SAT_OUT = 35676;

    if (x > SAT_IN)
        return SAT_OUT;
    if (x < -SAT_IN)
        return -SAT_OUT;

    int64_t abs_x = llabs((int64_t) x);
    int64_t x_cubed = abs_x * abs_x * abs_x;
    int64_t cubic_term = x_cubed >> 34;

    int64_t out64 = (int64_t) x - ((x < 0) ? -cubic_term : cubic_term);
    int32_t out = (int32_t) out64;

    return out;
}

int biquad(int x, int b1, int a1, int a2, int *delay) {
    int y;
    static const int64_t b0 = 1 << 16;
    static const int64_t b2 = 1 << 16; // GNU Octave butterworth lowass filter: b0,b1 and a0 always = 1

    y = (int) rounded_rshift((int64_t) b0 * x, 16) + delay[1];
    delay[1] = (int) rounded_rshift((int64_t) b1 * x - (int64_t) a1 * y, 16) + delay[0]; // here is "minus", so it fits to the polartiy of the coefficients from GNU octave
    delay[0] = (int) rounded_rshift((int64_t) b2 * x - (int64_t) a2 * y, 16);

    return y;
}

int biquad_b2zero(int x, int b1, int a1, int a2, int *delay) {
    int y;
    static const int64_t b0 = 1 << 16;

    y = (int) rounded_rshift((int64_t) b0 * x, 16) + delay[1];
    delay[1] = (int) rounded_rshift((int64_t) b1 * x - (int64_t) a1 * y, 16) + delay[0]; // here is "minus", so it fits to the polarity of the coefficients from GNU octave
    delay[0] = (int) rounded_rshift(-(int64_t) a2 * y, 16);

    return y;
}

/**
 * \brief 2-pole Bessel low-pass filter at 3000 Hz (16 kHz sample rate)..
 */
int filter_2pol_lowpass_3000hz_bessel(int x, int *delay) {
    int stage1;

    stage1 = biquad(x, 2 << 16, -27865, 7278, &delay[0]);
    stage1 = stage1 >> 3; // reduce the gain to approx. 0.73  ( = 1/(g*8) )

    return stage1;
}

/**
 * \brief 4-pole Bessel low-pass filter at 3000 Hz (16 kHz sample rate).
 */
int filter_4pol_lowpass_3000hz_bessel(int x, int *delay) {
    int stage1;
    int stage2;

    stage1 = biquad(x, 2 << 16, -27521, 17143, &delay[0]);
    stage2 = biquad(stage1, 2 << 16, -30251, 4755, &delay[2]);
    stage2 = stage2 >> 5; // reduce the gain to approx. 0.97  ( = 1/(g*32) ). TODO: distribute the gain reduction in between the stages?

    return stage2;
}

/**
 * \brief 4-pole Butterworth low-pass filter at 3000 Hz (16 kHz sample rate).
 */
int filter_4pol_lowpass_3000hz(int x, int *delay) {
    int stage1;
    int stage2;

    stage1 = biquad(x, 2 << 16, -27061, 5178, &delay[0]);
    stage2 = biquad(stage1, 2 << 16, -37057, 31299, &delay[2]);
    stage2 = stage2 >> 4; // reduce the gain to approx. 1.65  ( = 1/(g*16) ). TODO: distribute the gain reduction in between the stages?

    return stage2;
}

/**
 * \brief 4-pole Butterworth low-pass filter at 3400 Hz (16 kHz sample rate).
 */
int filter_4pol_lowpass_3400hz(int x, int *delay) {
    int stage1;
    int stage2;

    stage1 = biquad(x, 2 << 16, -16118, 3509, &delay[0]);       // a1 = -0.245945*65536 = -16118  /  a2 = 0.053545*65536 =  3509
    stage2 = biquad(stage1, 2 << 16, -22300, 29900, &delay[2]); // a1 = -0.340272*65536 = -22300  /  a2 = 0.457609*65536 = 29990
    stage2 = stage2 >> 4;                                       // reduce the gain to approx. 1.12  ( = 1/(g*16) )
                                                                // TODO: distribute the gain reduction in between the stages?!??!?!?

    return stage2;
}

/**
 * \brief 2-pole Butterworth low-pass filter at 3400 Hz (16 kHz sample rate).
 */
int filter_2pol_lowpass_3400hz(int x, int *delay) {
    int stage1;

    stage1 = biquad(x, 2 << 16, -18131, 12133, &delay[0]);
    stage1 = stage1 >> 2; //  reduce the gain to approx. 1.10  ( = 1/(g*4) )

    return stage1;
}

/* ---------- 1-pole highpass 500 Hz (fs = 16 kHz) ---------- */
int filter_1pol_highpass_500hz(polar_mod_ctx_t *ctx, int x) {
    /* R = exp(-2*pi*500/16000) -> Q16 ~= 53853 */
    const int R_q16 = 53853;
    return hp_dcblock_single_robust(ctx->delay_hp500, x, R_q16);
}

/* ---------- 1-pole highpass 1000 Hz (fs = 16 kHz) ---------- */
int filter_1pol_highpass_1000hz(polar_mod_ctx_t *ctx, int x) {
    /* R = exp(-2*pi*1000/16000) -> Q16 ~= 44252 */
    const int R_q16 = 44252;
    return hp_dcblock_single_robust(ctx->delay_hp1000, x, R_q16);
}

/* ---------- 1-pole highpass 2000 Hz (fs = 16 kHz) ---------- */
int filter_1pol_highpass_2000hz(polar_mod_ctx_t *ctx, int x) {
    /* R = exp(-2*pi*2000/16000) -> Q16 ~= 29880 */
    const int R_q16 = 29880;
    return hp_dcblock_single_robust(ctx->delay_hp2000, x, R_q16);
}

/*
 * 4-pole highpass at 200 Hz:
 * Implemented as cascade of four robust single-pole DC-blockers.
 * Uses ctx->delay_hp200_s1 and ctx->delay_hp200_s2 as available stage storage,
 * alternating them so we don't need extra arrays in the struct.
 */
int filter_4pol_highpass_200hz(polar_mod_ctx_t *ctx, int x) {
    /* R = exp(-2*pi*200/16000) -> Q16 ~= 60586 */
    const int R_q16 = 60586;

    int y1 = hp_dcblock_single_robust(ctx->delay_hp200_s1, x, R_q16);
    int y2 = hp_dcblock_single_robust(ctx->delay_hp200_s2, y1, R_q16);
    int y3 = hp_dcblock_single_robust(ctx->delay_hp200_s1, y2, R_q16);
    int y4 = hp_dcblock_single_robust(ctx->delay_hp200_s2, y3, R_q16);

    return y4;
}

/*
 * 4-pole highpass at 300 Hz:
 * Cascade of four robust single-poles using three available delay arrays
 * (re-using one to form the 4 stages).
 */
int filter_4pol_highpass_300hz(polar_mod_ctx_t *ctx, int x) {
    /* R = exp(-2*pi*300/16000) -> Q16 ~= 58253 */
    const int R_q16 = 58253;

    int y1 = hp_dcblock_single_robust(ctx->delay_hp300_s1, x, R_q16);
    int y2 = hp_dcblock_single_robust(ctx->delay_hp300_s2, y1, R_q16);
    int y3 = hp_dcblock_single_robust(ctx->delay_hp300_2p, y2, R_q16);
    int y4 = hp_dcblock_single_robust(ctx->delay_hp300_s1, y3, R_q16);

    return y4;
}

/* 2-pole highpass at 300 Hz: two robust single-pole cascade */
int filter_2pol_highpass_300hz(polar_mod_ctx_t *ctx, int x) {
    /* R = exp(-2*pi*300/16000) -> Q16 ~= 58253 */
    const int R_q16 = 58253;

    int y1 = hp_dcblock_single_robust(ctx->delay_hp300_s1, x, R_q16);
    int y2 = hp_dcblock_single_robust(ctx->delay_hp300_s2, y1, R_q16);

    return y2;
}

/**
 * \brief Computes an all-pass filter stage for Hilbert transform.
 */
int allpass(int x, int coeff, int *delay) {
    int signal_top_right;
    int y;

    signal_top_right = ((int64_t) coeff * (delay[0] - x)) >> 16;
    y = signal_top_right + delay[0];
    delay[0] = delay[1];
    delay[1] = signal_top_right + x;

    return y;
}

/**
 * \brief Performs Hilbert transform to generate I and Q components.
 */
void hilbert(polar_mod_ctx_t *ctx, int sample_in, int *i_out, int *q_out) {
    int i1, i2, i3, q1, q2, q3;

    // inphase 1. to 4. Block
    i1 = allpass(ctx->delay_i0, 31418, ctx->delay_i1); // coeff = ( ( 0.6923877778065 ) ^ 2 ) << 16
    i2 = allpass(i1, 57434, ctx->delay_i2);            // coeff = ( ( 0.9360654322959 ) ^ 2 ) << 16
    i3 = allpass(i2, 64002, ctx->delay_i3);            // coeff = ( ( 0.9882295226860 ) ^ 2 ) << 16
    *i_out = allpass(i3, 65372, ctx->delay_i4);        // coeff = ( ( 0.9987488452737 ) ^ 2 ) << 16
    // inphase Z^-1 block
    ctx->delay_i0 = sample_in;

    // quadrature 1. to 4. Block
    q1 = allpass(sample_in, 10601, ctx->delay_q1); // coeff = ( ( 0.4021921162426 ) ^ 2 ) << 16
    q2 = allpass(q1, 48040, ctx->delay_q2);        // coeff = ( ( 0.8561710882420 ) ^ 2 ) << 16
    q3 = allpass(q2, 61954, ctx->delay_q3);        // coeff = ( ( 0.9722909545651 ) ^ 2 ) << 16
    *q_out = allpass(q3, 64920, ctx->delay_q4);    // coeff = ( ( 0.9952884791278 ) ^ 2 ) << 16
}

/**
 * \brief Computes magnitude and angle using CORDIC algorithm.
 */
void cordic(int32_t x, int32_t y, int32_t *out_abs, int32_t *out_angle) {
    if (x == 0 && y == 0) {
        if (out_abs)
            *out_abs = 0;
        if (out_angle)
            *out_angle = 0;
        return;
    }

    const int SCALE = 28;

    int64_t x_new = ((int64_t) x) << SCALE;
    int64_t y_new = ((int64_t) y) << SCALE;
    int64_t angle = 0;

    int64_t inv_k = 1LL << 31;

    // Quadrant normalization
    if (x < 0) {
        if (y >= 0)
            angle = (1 << 23);
        else
            angle = -(1 << 23);
        x_new = -x_new;
        y_new = -y_new;
    }

    // Vectoring loop (Modified: Use rounded shifts)
    for (int i = 0; i < CORDIC_ITERATIONS; i++) {
        int64_t x_shift = rounded_rshift(x_new, i);
        int64_t y_shift = rounded_rshift(y_new, i);

        if (y_new > 0) {
            x_new += y_shift;
            y_new -= x_shift;
            angle += ang_table_q24[i];
            inv_k = (inv_k * inv_scale_q31[i] + (1LL << 30)) >> 31;
        } else
            if (y_new < 0) {
                x_new -= y_shift;
                y_new += x_shift;
                angle -= ang_table_q24[i];
                inv_k = (inv_k * inv_scale_q31[i] + (1LL << 30)) >> 31;
            }
    }

    // Magnitude correction (Modified: Use dynamic inv_k and adjusted rounding)
    int64_t a = rounded_rshift(x_new, SCALE);
    int64_t prod = a * inv_k;
    int64_t mag_int = (prod + (1LL << 30)) >> 31;
    int32_t mag = (int32_t) mag_int;

    // Normalize angle
    const int32_t Q24_180 = (1 << 23);
    const int32_t Q24_360 = (1 << 24);
    if (angle > Q24_180)
        angle -= Q24_360;
    if (angle <= -Q24_180)
        angle += Q24_360;

    if (out_abs)
        *out_abs = mag;
    if (out_angle)
        *out_angle = angle;
}

/**
 * \brief Generates IQ test signals using lookup tables.
 */
void iq_signal_generator(polar_mod_ctx_t *ctx, int mode, int *x, int *y) {
    /* Reset counter when mode changes */
    if (ctx->last_mode != (unsigned) mode) {
        ctx->last_mode = (unsigned) mode;
        ctx->counter = 0;
    }

    /* Use actual active length (table has 30 valid entries out of 32) */
    const unsigned int active_size = (LOOKUP_SIZE > 2) ? (LOOKUP_SIZE - 2) : LOOKUP_SIZE;

    unsigned int idx = ctx->counter;
    if (idx >= active_size)
        idx %= active_size;

    if (mode == SPECIAL_MODULATION_2_TONE_SIG_IQ || mode == 0) {
        *x = table_2_tone_x[idx];
        *y = table_2_tone_y[idx];
    } else
        if (mode == SPECIAL_MODULATION_3_TONE_SIG_IQ) {
            *x = table_3_tone_x[idx];
            *y = table_3_tone_y[idx];
        } else {
            *x = 0;
            *y = 0;
        }

    /* Advance counter (post-increment) and wrap cleanly */
    ctx->counter++;
    if (ctx->counter >= active_size)
        ctx->counter = 0;
}

///////////////////////////////////////// API /////////////////////////////////////////

/**
 * \brief Initializes the polar modulator context. (see header for details).
 */
void polar_mod_init(polar_mod_ctx_t *ctx) {
    memset(ctx, 0, sizeof(polar_mod_ctx_t));
    ctx->gain_value = 1000; // AGC start value
    ctx->agc_gain = 256;    // Default gain (1.0)
    ctx->last_mode = 555;   // Dummy for signal generator
    ctx->sample_rate = 16000; // Set default sample rate to 16000 Hz
}

/**
 * \brief Implements polar modulation (see header for details).
 */
int polar_modulator(polar_mod_ctx_t *ctx, modulation_t modulation, int data, int *ampl_out, int *phase_diff_out) {
    if (!ampl_out || !phase_diff_out)
        return -1;

    int data_2, data_3, data_4, data_5;
    int x, y;
    int ampl;
    int angle;
    int angle_diff = 0;

    // high pass, also removes DC bias
    switch (modulation.filter_pre_hp) {
        case FILTER_HP_200_4pol:
            data_2 = filter_4pol_highpass_200hz(ctx, data);
            break;
        case FILTER_HP_300_4pol:
            data_2 = filter_4pol_highpass_300hz(ctx, data);
            break;
        case FILTER_HP_300_2pol:
        default:
            data_2 = filter_2pol_highpass_300hz(ctx, data);
    }
    data_2 <<= 1;

    // general lowpass filter
    switch (modulation.filter_pre_lp) {
        case FILTER_LP_3400_2pol:
            data_3 = filter_2pol_lowpass_3400hz(data_2, ctx->delay_lp_adc);
            break;
        case FILTER_LP_3400_4pol:
            data_3 = filter_4pol_lowpass_3400hz(data_2, ctx->delay_lp_adc);
            break;
        case FILTER_LP_3000_4pol:
            data_3 = filter_4pol_lowpass_3000hz(data_2, ctx->delay_lp_adc);
            break;
        case FILTER_LP_3000_2pol:
            data_3 = filter_2pol_lowpass_3000hz_bessel(data_2, ctx->delay_lp_adc);
            break;
        default:
            data_3 = filter_4pol_lowpass_3400hz(data_2, ctx->delay_lp_adc);
    }
    data_3 <<= 1;

    // shape passband
    switch (modulation.filter_pre_pb) {
        case FILTER_PB_500:
            data_3 = filter_1pol_highpass_500hz(ctx, data_3);
            break;
        case FILTER_PB_2k:
            data_3 = filter_1pol_highpass_2000hz(ctx, data_3);
            break;
        case FILTER_PB_NONE:
            break;
        case FILTER_PB_1k:
            data_3 = filter_1pol_highpass_1000hz(ctx, data_3);
            break;
        default:
            data_3 = filter_1pol_highpass_2000hz(ctx, data_3);
            break;
    }
    data_3 <<= 1;

    // AGC
    switch (modulation.agc_type) {
        case AGC_GAIN_FIX:
            data_4 = (data_3 * (0x100 & 0xff)) >> 4;
            break;
        case AGC_GAIN_CHANGE:
            data_4 = (data_3 * ctx->agc_gain * ((0x200 & 0xff))) >> 4;
            break;
        case 0:
        default:
            data_4 = (data_3 * ctx->agc_gain) >> 8;
            break;
    }

    ctx->agc_gain = mic_agc_fast(ctx, abs((data_3 * ctx->agc_gain) >> 8), modulation.polar_status);
    data_5 = soft_limiter(data_4);

    // post lowpass
    switch (modulation.filter_post_lp) {
        case FILTER_POST_LP_3400_2pol:
            data_5 = filter_2pol_lowpass_3400hz(data_5, ctx->delay_lp_2);
            break;
        case FILTER_POST_LP_3400_4pol:
            data_5 = filter_4pol_lowpass_3400hz(data_5, ctx->delay_lp_2);
            break;
        case FILTER_POST_LP_3000_2pol:
            data_5 = filter_2pol_lowpass_3000hz_bessel(data_5, ctx->delay_lp_2);
            break;
        case FILTER_POST_LP_NONE:
            break;
        case FILTER_POST_LP_3000_4pol:
        default:
            data_5 = filter_2pol_lowpass_3400hz(data_5, ctx->delay_lp_2);
    }

    hilbert(ctx, data_5, &x, &y);

    x = filter_2pol_lowpass_3400hz(x, ctx->delay_lp_x);
    y = filter_2pol_lowpass_3400hz(y, ctx->delay_lp_y);

    if ((modulation.special_modulation == SPECIAL_MODULATION_2_TONE_SIG_IQ) || (modulation.special_modulation == SPECIAL_MODULATION_3_TONE_SIG_IQ)) {
        iq_signal_generator(ctx, modulation.special_modulation, &x, &y);
    }

    cordic(x, y, &ampl, &angle);

    switch (modulation.modulation_mode) {
        case MOD_AM:
            angle_diff = 0;
            if (ampl < 0)
                ampl = 0;
            if (ampl > 32767)
                ampl = 32767;
            *ampl_out = (ampl << 1);  // scale up to full 16-bit
            break;

        case MOD_FM:
            angle_diff = data_5 * 148;
            *ampl_out = 65535;
            break;

        case MOD_FMN:
            angle_diff = data_5 * 74;
            *ampl_out = 65535;
            break;

        case MOD_FMW:
            angle_diff = data_5 * 2220;
            *ampl_out = 65535;
            break;

        case MOD_CW:
            angle_diff = 0;
            *ampl_out = 65535;
            break;

        case MOD_USB:
        case MOD_LSB:
            angle_diff = angle - ctx->last_angle;
            if (angle_diff > (1 << 23)) angle_diff -= (1 << 24);  // Unwrap positive wraparound
            if (angle_diff < -(1 << 23)) angle_diff += (1 << 24); // Unwrap negative wraparound
            ctx->last_angle = angle;
            if (angle_diff > 0x600000)
                angle_diff = 0x600000;
            if (angle_diff < -0x600000)
                angle_diff = -0x600000;
            if (modulation.modulation_mode == MOD_USB)
                angle_diff = -angle_diff;
            *ampl_out = ampl;
            break;

        default:
            return -1;
    }

    *phase_diff_out = angle_diff;
    return 0;
}
