/*
 * Copyright 2025 Emiliano Gonzalez (egonzalez . hiperion @ gmail . com))
 * * Project Site:  *
 *
 * This is based on other projects:
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

#include <stdio.h>
#include <string.h>
#include "polar_mod.h"

/* Extended tests to ensure every function in polar_mod.c is executed.
   Designed for a host build (tests) and to exercise integer paths suitable
   for a 32-bit microcontroller without FPU. */

void test_cordic(void);
void test_hilbert_and_iq(polar_mod_ctx_t *ctx);
void test_filters(polar_mod_ctx_t *ctx);
void test_agc_and_limiter(polar_mod_ctx_t *ctx);
void test_iq_signal_generator(polar_mod_ctx_t *ctx);
void test_modulation_am_pm(polar_mod_ctx_t *ctx);

void test_filters(polar_mod_ctx_t *ctx) {
    printf("Running test_filters...\n");
    int delay[2] = {0,0};
    int out;

    /* biquad (general) */
    out = biquad(1000, (int)(0.5*(1<<16)), (int)(0.2*(1<<16)), (int)(0.1*(1<<16)), delay);
    printf("biquad -> %d\n", out);

    /* biquad_b2zero */
    delay[0]=delay[1]=0;
    out = biquad_b2zero(1000, (int)(0.5*(1<<16)), (int)(0.2*(1<<16)), (int)(0.1*(1<<16)), delay);
    printf("biquad_b2zero -> %d\n", out);

    /* 2-pole Bessel lowpass 3000Hz */
    delay[0]=delay[1]=0;
    out = filter_2pol_lowpass_3000hz_bessel(1000, delay);
    printf("filter_2pol_lowpass_3000hz_bessel -> %d\n", out);

    /* 4-pole Bessel lowpass 3000Hz */
    int d4[4] = {0,0,0,0};
    out = filter_4pol_lowpass_3000hz_bessel(1000, d4);
    printf("filter_4pol_lowpass_3000hz_bessel -> %d\n", out);

    /* other lowpass variants */
    d4[0]=d4[1]=d4[2]=d4[3]=0;
    out = filter_4pol_lowpass_3000hz(1000, d4);
    printf("filter_4pol_lowpass_3000hz -> %d\n", out);

    d4[0]=d4[1]=d4[2]=d4[3]=0;
    out = filter_4pol_lowpass_3400hz(1000, d4);
    printf("filter_4pol_lowpass_3400hz -> %d\n", out);

    int d2[2] = {0,0};
    out = filter_2pol_lowpass_3400hz(1000, d2);
    printf("filter_2pol_lowpass_3400hz -> %d\n", out);

    /* highpass filters that require ctx */
    ctx->delay_hp500[0]=ctx->delay_hp500[1]=0;
    out = filter_1pol_highpass_500hz(ctx, 1000);
    printf("filter_1pol_highpass_500hz -> %d\n", out);

    ctx->delay_hp500[0]=ctx->delay_hp500[1]=0;
    out = filter_1pol_highpass_1000hz(ctx, 1000);
    printf("filter_1pol_highpass_1000hz -> %d\n", out);

    ctx->delay_hp500[0]=ctx->delay_hp500[1]=0;
    out = filter_1pol_highpass_2000hz(ctx, 1000);
    printf("filter_1pol_highpass_2000hz -> %d\n", out);

    /* 4-pole highpass */
    ctx->delay_lp_adc[0]=ctx->delay_lp_adc[1]=ctx->delay_lp_adc[2]=ctx->delay_lp_adc[3]=0; /* reused array for zeroing */
    out = filter_4pol_highpass_200hz(ctx, 1000);
    printf("filter_4pol_highpass_200hz -> %d\n", out);

    ctx->delay_lp_adc[0]=ctx->delay_lp_adc[1]=ctx->delay_lp_adc[2]=ctx->delay_lp_adc[3]=0; /* reused array for zeroing */
    out = filter_4pol_highpass_300hz(ctx, 1000);
    printf("filter_4pol_highpass_300hz -> %d\n", out);

    ctx->delay_hp300_2p[0]=ctx->delay_hp300_2p[1]=0;
    out = filter_2pol_highpass_300hz(ctx, 1000);
    printf("filter_2pol_highpass_300hz -> %d\n", out);

    /* allpass */
    int delay_ap[2] = {0,0};
    out = allpass(1000, (int)(0.7*(1<<16)), delay_ap);
    printf("allpass -> %d\n", out);

    printf("test_filters done.\n");
}

void test_iq_signal_generator(polar_mod_ctx_t *ctx) {
    printf("Running test_iq_signal_generator...\n");
    int x,y;
    /* call mode 0,1,2 to exercise branches */
    iq_signal_generator(ctx, 0, &x, &y);
    printf("iq mode 0 -> %d,%d\n", x,y);
    iq_signal_generator(ctx, 1, &x, &y);
    printf("iq mode 1 -> %d,%d\n", x,y);
    iq_signal_generator(ctx, 2, &x, &y);
    printf("iq mode 2 -> %d,%d\n", x,y);
    printf("test_iq_signal_generator done.\n");
}

/* Minimal versions of other tests; they already call multiple functions in polar_mod.c */
void test_cordic(void) {
    printf("Running test_cordic...\n");
    int32_t a,b,mag,ang;
    a = 10000; b = 5000;
    cordic(a,b,&mag,&ang);
    printf("cordic -> mag=%d ang=%d\n", mag, ang);
    printf("test_cordic done.\n");
}

void test_hilbert_and_iq(polar_mod_ctx_t *ctx) {
    printf("Running test_hilbert_and_iq...\n");
    int i,q;
    hilbert(ctx, 12345, &i, &q);
    printf("hilbert -> i=%d q=%d\n", i, q);
    printf("test_hilbert_and_iq done.\n");
}

void test_agc_and_limiter(polar_mod_ctx_t *ctx) {
    printf("Running test_agc_and_limiter...\n");
    int out;
    out = mic_agc_fast(ctx, 20000, 0);
    printf("mic_agc_fast -> %d\n", out);
    out = soft_limiter(15000);
    printf("soft_limiter -> %d\n", out);
    printf("test_agc_and_limiter done.\n");
}

void test_modulation_am_pm(polar_mod_ctx_t *ctx) {
    printf("Running test_modulation_am_pm...\n");
    int ampl, phase;

    modulation_t mod;
    memset(&mod, 0, sizeof(mod));

    /* SPECIAL_MODULATION_FM_DIRECT */
    mod.special_modulation = SPECIAL_MODULATION_FM_DIRECT;
    modulation_am_pm(ctx, mod, 100, &ampl, &phase);
    printf("modulation_am_pm (SPECIAL_MODULATION_FM_DIRECT) -> ampl=%d phase=%d\n", ampl, phase);

    /* SPECIAL_MODULATION_AM_DIRECT */
    memset(&mod, 0, sizeof(mod));
    mod.special_modulation = SPECIAL_MODULATION_AM_DIRECT;
    modulation_am_pm(ctx, mod, 100, &ampl, &phase);
    printf("modulation_am_pm (SPECIAL_MODULATION_AM_DIRECT) -> ampl=%d phase=%d\n", ampl, phase);

    /* SPECIAL_MODULATION_2_TONE_SIG */
    memset(&mod, 0, sizeof(mod));
    mod.special_modulation = SPECIAL_MODULATION_2_TONE_SIG;
    modulation_am_pm(ctx, mod, 100, &ampl, &phase);
    printf("modulation_am_pm (SPECIAL_MODULATION_2_TONE_SIG) -> ampl=%d phase=%d\n", ampl, phase);

    printf("test_modulation_am_pm done.\n");
}

int main(void) {
    polar_mod_ctx_t ctx;
    memset(&ctx,0,sizeof(ctx));
    polar_mod_init(&ctx);

    test_cordic();
    test_hilbert_and_iq(&ctx);
    test_filters(&ctx);
    test_agc_and_limiter(&ctx);
    test_iq_signal_generator(&ctx);
    test_modulation_am_pm(&ctx);

    printf("All tests finished.\n");
    return 0;
}
