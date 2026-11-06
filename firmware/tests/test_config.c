/* Calibration (mrad <-> ticks, offset, direction) and CONFIG key validation. */
#include <math.h>

#include "armfw/config.h"
#include "armfw/proto.h"
#include "tt.h"

TT_TEST(defaults_match_kinematics_yaml) {
    armfw_config_t c;
    config_defaults(&c);
    static const int lo[6] = {-2600, -1800, -2700, -1900, -3000, 0};
    static const int hi[6] = {2600, 750, 2300, 1900, 3000, 1600};
    static const int vm[6] = {3000, 2500, 3000, 3500, 4000, 4000};
    for (int j = 0; j < 6; j++) {
        TT_EQ(c.joint[j].limit_min_mrad, lo[j]);
        TT_EQ(c.joint[j].limit_max_mrad, hi[j]);
        TT_EQ(c.joint[j].vmax_mrad_s, vm[j]);
        TT_EQ(c.joint[j].direction, 1);
        TT_EQ(c.joint[j].offset_ticks, 0);
        TT_EQ(c.joint[j].servo_id, j + 1);
    }
    TT_EQ(c.wd_hold_ms, 100);
    TT_EQ(c.wd_off_ms, 500);
}

TT_TEST(ticks_conversion) {
    joint_cfg_t j = {.offset_ticks = 0, .direction = 1};
    TT_EQ(config_mrad_to_ticks(&j, 0), 2048);
    /* +pi rad = half a turn = 2048 ticks */
    TT_EQ(config_mrad_to_ticks(&j, 3141.5927f), 4095); /* 4096 clamps to 4095 */
    TT_EQ(config_mrad_to_ticks(&j, -3141.5927f), 0);
    TT_EQ(config_mrad_to_ticks(&j, 1570.7963f), 3072);
    TT_NEAR(config_ticks_to_mrad(&j, 3072), 1570.796, 1e-2);
    TT_NEAR(config_ticks_to_mrad(&j, 2049), 1.5339808, 1e-5);
    j.direction = -1;
    TT_EQ(config_mrad_to_ticks(&j, 1570.7963f), 1024);
    TT_NEAR(config_ticks_to_mrad(&j, 1024), 1570.796, 1e-2);
    j.offset_ticks = -100;
    TT_EQ(config_mrad_to_ticks(&j, 0), 1948);
    TT_NEAR(config_ticks_to_mrad(&j, 1948), 0, 1e-6);
    TT_NEAR(config_tickrate_to_mrad_s(&j, 100), -153.39808, 1e-3);
}

TT_TEST(ticks_roundtrip_property) {
    tt_rng_t r = {5};
    for (int i = 0; i < 5000; i++) {
        joint_cfg_t j = {.offset_ticks = tt_randi(&r, -300, 300), .direction = (int8_t)(tt_rand(&r) & 1 ? 1 : -1)};
        float mrad = (float)tt_randf(&r, -2500, 2500); /* |ticks| <= 1630 + 300: never clamped */
        uint16_t t = config_mrad_to_ticks(&j, mrad);
        float back = config_ticks_to_mrad(&j, t);
        TT_ASSERT_MSG(fabs(back - mrad) <= CFG_MRAD_PER_TICK / 2 + 1e-3, "mrad %f -> %u -> %f", mrad, t, back);
    }
}

TT_TEST(config_apply_validation) {
    armfw_config_t c;
    config_defaults(&c);
    TT_EQ(config_apply(&c, 0, PROTO_CFG_OFFSET_TICKS, 4095, false), 0);
    TT_EQ(c.joint[0].offset_ticks, 4095);
    TT_EQ(config_apply(&c, 0, PROTO_CFG_OFFSET_TICKS, 4096, false), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0, PROTO_CFG_OFFSET_TICKS, 10, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 1, PROTO_CFG_DIRECTION, -1, false), 0);
    TT_EQ(c.joint[1].direction, -1);
    TT_EQ(config_apply(&c, 1, PROTO_CFG_DIRECTION, 0, false), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 1, PROTO_CFG_DIRECTION, 1, true), PROTO_ERR_BAD_VALUE);
    /* limits must stay ordered */
    TT_EQ(config_apply(&c, 2, PROTO_CFG_LIMIT_MIN, 2700, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 2, PROTO_CFG_LIMIT_MIN, -1000, true), 0);
    TT_EQ(config_apply(&c, 2, PROTO_CFG_LIMIT_MAX, -1000, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 2, PROTO_CFG_LIMIT_MAX, 40000, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 2, PROTO_CFG_LIMIT_MAX, 1000, true), 0);
    TT_EQ(c.joint[2].limit_min_mrad, -1000);
    TT_EQ(c.joint[2].limit_max_mrad, 1000);
    TT_EQ(config_apply(&c, 3, PROTO_CFG_VMAX, 0, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 3, PROTO_CFG_VMAX, 1500, true), 0);
    TT_EQ(config_apply(&c, 4, PROTO_CFG_CURRENT_LIMIT, 0, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 4, PROTO_CFG_CURRENT_LIMIT, 1200, true), 0);
    TT_EQ(c.joint[4].current_limit_ma, 1200);
    /* joint index and key class */
    TT_EQ(config_apply(&c, 6, PROTO_CFG_VMAX, 100, false), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0xFF, PROTO_CFG_VMAX, 100, false), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0, PROTO_CFG_WD_HOLD_MS, 100, false), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0, 0x07, 1, false), PROTO_ERR_UNKNOWN_KEY);
    TT_EQ(config_apply(&c, 0xFF, 0x12, 1, false), PROTO_ERR_UNKNOWN_KEY);
    /* watchdog: hold < off */
    TT_EQ(config_apply(&c, 0xFF, PROTO_CFG_WD_HOLD_MS, 500, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0xFF, PROTO_CFG_WD_HOLD_MS, 0, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0xFF, PROTO_CFG_WD_HOLD_MS, 50, true), 0);
    TT_EQ(config_apply(&c, 0xFF, PROTO_CFG_WD_OFF_MS, 50, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0xFF, PROTO_CFG_WD_OFF_MS, 60001, true), PROTO_ERR_BAD_VALUE);
    TT_EQ(config_apply(&c, 0xFF, PROTO_CFG_WD_OFF_MS, 2000, true), 0);
    TT_EQ(c.wd_hold_ms, 50);
    TT_EQ(c.wd_off_ms, 2000);
}

TT_MAIN()
