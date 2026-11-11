#include "armfw/analog.h"

#include <string.h>

#define KELVIN_0C 273.15f
#define T25_K 298.15f

void analog_default_params(analog_params_t *p) {
    p->vdda_nominal_mv = 3300;
    p->div_top_ohm = 100000u;
    p->div_bot_ohm = 15000u;
    p->shunt_uohm = 3000u;
    p->isense_gain = 50u;
    p->isense_offset_mv = 0;
    p->ntc_r25_ohm = 10000u;
    p->ntc_beta_k = 3380u;
    p->ntc_pullup_ohm = 10000u;
}

void analog_init(analog_t *a, const analog_params_t *p) {
    memset(a, 0, sizeof(*a));
    a->p = *p;
    a->vdda_mv = p->vdda_nominal_mv;
}

float analog_lnf(float x) {
    if (!(x > 0.0f)) return -1e30f;
    int k = 0;
    while (x > 1.41421356f) {
        x *= 0.5f;
        k++;
    }
    while (x < 0.70710678f) {
        x *= 2.0f;
        k--;
    }
    /* ln(x) = 2 atanh(t), t = (x-1)/(x+1), |t| <= 0.1716 */
    float t = (x - 1.0f) / (x + 1.0f);
    float t2 = t * t;
    float s = t * (1.0f + t2 * (1.0f / 3.0f + t2 * (1.0f / 5.0f + t2 * (1.0f / 7.0f + t2 * (1.0f / 9.0f)))));
    return 2.0f * s + (float)k * 0.69314718f;
}

float analog_ntc_temp_c(const analog_params_t *p, float code) {
    if (code <= 0.5f) return 1e3f;                        /* shorted to GND: absurdly hot */
    if (code >= (float)ADC_FULL_SCALE - 0.5f) return -1e3f; /* open: absurdly cold */
    float r = (float)p->ntc_pullup_ohm * code / ((float)ADC_FULL_SCALE - code);
    float inv_t = 1.0f / T25_K + analog_lnf(r / (float)p->ntc_r25_ohm) / (float)p->ntc_beta_k;
    return 1.0f / inv_t - KELVIN_0C;
}

static float div_ratio(const analog_params_t *p) {
    return (float)(p->div_top_ohm + p->div_bot_ohm) / (float)p->div_bot_ohm;
}

/* mA per pin mV */
static float isense_ma_per_mv(const analog_params_t *p) {
    return 1e6f / ((float)p->shunt_uohm * (float)p->isense_gain);
}

static uint16_t code_from_pin_mv(float pin_mv, uint32_t vdda_mv) {
    float c = pin_mv * (float)ADC_FULL_SCALE / (float)vdda_mv;
    c = armfw_clampf(c, 0.0f, (float)ADC_FULL_SCALE);
    return (uint16_t)(c + 0.5f);
}

uint16_t analog_code_for_bus_mv(const analog_params_t *p, uint32_t mv, uint32_t vdda_mv) {
    return code_from_pin_mv((float)mv / div_ratio(p), vdda_mv);
}

uint16_t analog_code_for_bus_ma(const analog_params_t *p, int32_t ma, uint32_t vdda_mv) {
    if (ma < 0) ma = 0; /* unidirectional amplifier saturates at its ground rail */
    return code_from_pin_mv((float)ma / isense_ma_per_mv(p) + (float)p->isense_offset_mv, vdda_mv);
}

uint16_t analog_code_for_temp_c(const analog_params_t *p, float temp_c) {
    /* R(T) = R25 * exp(B (1/T - 1/T25)); exp via repeated squaring-free series is overkill for a
     * test helper, so invert the forward model numerically (bisection on the code). */
    float lo = 1.0f, hi = (float)ADC_FULL_SCALE - 1.0f;
    for (int i = 0; i < 40; i++) {
        float mid = 0.5f * (lo + hi);
        /* temperature falls as the code rises (NTC on the low side) */
        if (analog_ntc_temp_c(p, mid) > temp_c)
            lo = mid;
        else
            hi = mid;
    }
    return (uint16_t)(0.5f * (lo + hi) + 0.5f);
}

static int32_t iir(int32_t y, uint16_t x, unsigned shift) {
    int32_t xq = (int32_t)x << 8;
    return y + (xq - y) / (int32_t)(1u << shift);
}

void analog_update(analog_t *a, const adc_raw_t *raw) {
    if (!raw) {
        if (a->stale_ms < 0xFFFFFFFFu) a->stale_ms++;
        if (a->stale_ms >= ANALOG_STALE_MS) {
            /* Lost the ADC: report 0 V (raises BUS_UNDERVOLT, a visible sensing problem) and 0 A
             * (cannot trip the bus overcurrent on stale data). */
            a->valid = false;
            a->bus_mv = 0;
            a->bus_ma = 0;
            a->vin_mv = 0;
            a->ntc_ok = false;
        }
        return;
    }
    a->stale_ms = 0;
    a->samples++;
    bool have_vref = raw->vrefint != 0 && raw->vrefint_cal != 0;
    if (!a->primed) {
        a->f_isense = (int32_t)raw->isense << 8;
        a->f_vservo = (int32_t)raw->vservo << 8;
        a->f_vin = (int32_t)raw->vin << 8;
        a->f_ntc = (int32_t)raw->ntc << 8;
        a->f_vref = (int32_t)raw->vrefint << 8;
        a->primed = true;
    } else {
        a->f_isense = iir(a->f_isense, raw->isense, ANALOG_VI_SHIFT);
        a->f_vservo = iir(a->f_vservo, raw->vservo, ANALOG_VI_SHIFT);
        a->f_vin = iir(a->f_vin, raw->vin, ANALOG_VI_SHIFT);
        a->f_ntc = iir(a->f_ntc, raw->ntc, ANALOG_NTC_SHIFT);
        if (have_vref) a->f_vref = iir(a->f_vref, raw->vrefint, ANALOG_VREF_SHIFT);
    }

    const analog_params_t *p = &a->p;
    float vdda = (float)p->vdda_nominal_mv;
    if (have_vref && a->f_vref > 0) {
        /* VREFINT_CAL was taken at VDDA = 3.0 V */
        vdda = 3000.0f * (float)raw->vrefint_cal * 256.0f / (float)a->f_vref;
        vdda = armfw_clampf(vdda, 2000.0f, 3600.0f);
    }
    a->vdda_mv = (uint16_t)(vdda + 0.5f);
    float mv_per_q8 = vdda / ((float)ADC_FULL_SCALE * 256.0f);

    float ratio = div_ratio(p);
    float bus_mv = (float)a->f_vservo * mv_per_q8 * ratio;
    float vin_mv = (float)a->f_vin * mv_per_q8 * ratio;
    a->bus_mv = (uint16_t)armfw_clampf(bus_mv + 0.5f, 0.0f, 65535.0f);
    a->vin_mv = (uint16_t)armfw_clampf(vin_mv + 0.5f, 0.0f, 65535.0f);

    float pin_mv = (float)a->f_isense * mv_per_q8 - (float)p->isense_offset_mv;
    if (pin_mv < 0.0f) pin_mv = 0.0f;
    a->bus_ma = armfw_f2i16(pin_mv * isense_ma_per_mv(p));

    float t = analog_ntc_temp_c(p, (float)a->f_ntc * (1.0f / 256.0f));
    int32_t tdc = (int32_t)armfw_f2i16(armfw_clampf(t * 10.0f, -32000.0f, 32000.0f));
    a->ntc_ok = tdc >= ANALOG_NTC_MIN_DC && tdc <= ANALOG_NTC_MAX_DC;
    a->board_temp_dc = (int16_t)tdc;
    a->valid = true;
}
