/* Board analog front end: servo-rail voltage divider, INA240 servo-rail current, protected-input
 * voltage divider and the board NTC. The HAL only delivers raw ADC codes (hal_port.adc_read);
 * everything else (scaling, filtering, staleness, NTC linearisation) lives here so it is unit
 * tested on the host.
 *
 * Circuit (hardware/board/schematic.py, hardware/board/out/mcu_pins.md):
 *   VSERVO_ADC  PA1 = ADC12_IN2   V12_SERVO -- 100k -- pin -- 15k -- GND        (ratio 115/15)
 *   VIN_ADC     PC1 = ADC12_IN7   V12 (protected input) through the same 100k/15k
 *   ISENSE_ADC  PA0 = ADC12_IN1   INA240A2 (gain 50 V/V), 3 mOhm high-side shunt, REF1/REF2 = GND
 *                                 (unidirectional: 0 A = 0 V, 0.15 V/A, full scale ~22 A)
 *   NTC_ADC     PC0 = ADC12_IN6   +3V3A -- 10k -- pin -- NTC 10k B3380 (Murata NCP18XH103F03RB) -- GND
 * VREF+ is +3V3A, so the NTC reading is ratiometric (independent of VDDA). The divider and shunt
 * readings scale with VDDA; when the target supplies VREFINT and its factory calibration word the
 * actual VDDA is used, otherwise the nominal 3300 mV.
 */
#ifndef ARMFW_ANALOG_H
#define ARMFW_ANALOG_H

#include "armfw/common.h"

#define ADC_FULL_SCALE 4095u
#define ANALOG_STALE_MS 10u   /* no new conversion for this long -> readings invalid */
#define ANALOG_VI_SHIFT 1u    /* bus V/I IIR: y += (x - y) / 2 per sample (tau ~1.4 ms at 1 kHz) */
#define ANALOG_NTC_SHIFT 6u   /* NTC IIR: y += (x - y) / 64 per sample (tau ~64 ms) */
#define ANALOG_VREF_SHIFT 4u  /* VREFINT IIR */
#define ANALOG_NTC_MIN_DC (-400) /* plausible board temperature range, deci-degC; outside = sensor */
#define ANALOG_NTC_MAX_DC 1500   /* open/short (no overtemp trip on a broken sensor) */

/* Raw codes, 12-bit right aligned (the target may oversample in hardware and shift back to 12
 * bits). vrefint / vrefint_cal: VREFINT conversion and the factory VREFINT_CAL word measured at
 * 3.0 V; either 0 = not available (nominal VDDA is used). */
typedef struct {
    uint16_t isense;
    uint16_t vservo;
    uint16_t vin;
    uint16_t ntc;
    uint16_t vrefint;
    uint16_t vrefint_cal;
} adc_raw_t;

typedef struct {
    uint16_t vdda_nominal_mv;
    uint32_t div_top_ohm, div_bot_ohm; /* both rail dividers */
    uint32_t shunt_uohm;
    uint16_t isense_gain;      /* V/V */
    int16_t isense_offset_mv;  /* amplifier output at 0 A, at the pin (calibration, default 0) */
    uint32_t ntc_r25_ohm;
    uint32_t ntc_beta_k;
    uint32_t ntc_pullup_ohm;
} analog_params_t;

typedef struct {
    analog_params_t p;
    /* IIR states, codes in Q8 */
    int32_t f_isense, f_vservo, f_vin, f_ntc, f_vref;
    bool primed;
    uint32_t samples;
    uint32_t stale_ms;
    /* outputs */
    bool valid;          /* fresh conversions within ANALOG_STALE_MS */
    uint16_t vdda_mv;
    uint16_t bus_mv;     /* servo rail */
    int16_t bus_ma;      /* servo rail current (>= 0: the INA240 is wired unidirectional) */
    uint16_t vin_mv;     /* protected 12 V input */
    int16_t board_temp_dc; /* deci-degC */
    bool ntc_ok;
} analog_t;

void analog_default_params(analog_params_t *p);
void analog_init(analog_t *a, const analog_params_t *p);
/* Once per control step. raw == NULL means no new conversion this step. */
void analog_update(analog_t *a, const adc_raw_t *raw);

/* Pure conversions, exposed for tests and for the sim (which generates codes from physics). */
float analog_ntc_temp_c(const analog_params_t *p, float code);
float analog_lnf(float x); /* natural log, x > 0, |rel err| < 1e-6, no libm */
/* Inverse helpers used by the sim: physical value -> ideal 12-bit code at the given VDDA. */
uint16_t analog_code_for_bus_mv(const analog_params_t *p, uint32_t mv, uint32_t vdda_mv);
uint16_t analog_code_for_bus_ma(const analog_params_t *p, int32_t ma, uint32_t vdda_mv);
uint16_t analog_code_for_temp_c(const analog_params_t *p, float temp_c);

#endif
