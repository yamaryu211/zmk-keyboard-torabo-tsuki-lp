/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_acceleration

#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct scroll_acceleration_config {
    int32_t decay_ms;
    int32_t max_acceleration;
    int32_t min_speed_threshold;
};

struct scroll_acceleration_data {
    int64_t last_event_time;
    int32_t last_movement;
};

/**
 * Sigmoid-based acceleration function
 * Returns acceleration multiplier based on speed
 *
 * Formula: acceleration = 1.0 + (max_accel - 1.0) * sigmoid(speed)
 * where sigmoid(x) = 1 / (1 + exp(-0.5 * (x - 8)))
 */
static float calculate_acceleration(float speed, float sensitivity, float max_acceleration) {
    float adjusted_speed = speed * (sensitivity / 2.5f);
    float sigmoid = 1.0f / (1.0f + expf(-0.5f * (adjusted_speed - 8.0f)));
    float acceleration = 1.0f + (max_acceleration - 1.0f) * sigmoid;
    return acceleration;
}

static int scroll_acceleration_handle_event(const struct device *dev,
                                            struct input_event *event,
                                            uint32_t param1,
                                            uint32_t param2,
                                            struct zmk_input_processor_state *state) {
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    const struct scroll_acceleration_config *cfg = dev->config;
    struct scroll_acceleration_data *data = (struct scroll_acceleration_data *)dev->data;

    int64_t current_time = k_uptime_get();
    int64_t delta_time = data->last_event_time > 0 ?
                         current_time - data->last_event_time : 0;

    int32_t movement = abs(event->value);
    int32_t sensitivity = param1;

    if (sensitivity < 1) sensitivity = 1;
    if (sensitivity > 10) sensitivity = 10;

    if (movement <= cfg->min_speed_threshold) {
        data->last_event_time = current_time;
        data->last_movement = movement;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (delta_time > 0 && delta_time < cfg->decay_ms) {
        float speed = (float)movement / (float)delta_time;
        float acceleration = calculate_acceleration(speed, (float)sensitivity,
                                                    (float)cfg->max_acceleration);
        int32_t accelerated_value = (int32_t)(event->value * acceleration);

        LOG_DBG("Scroll accel: val=%d, speed=%.2f, accel=%.2f, result=%d",
                event->value, (double)speed, (double)acceleration, accelerated_value);

        event->value = accelerated_value;
    }

    data->last_event_time = current_time;
    data->last_movement = movement;

    return ZMK_INPUT_PROC_CONTINUE;
}

static int scroll_acceleration_init(const struct device *dev) {
    struct scroll_acceleration_data *data = (struct scroll_acceleration_data *)dev->data;
    data->last_event_time = 0;
    data->last_movement = 0;
    return 0;
}

static const struct zmk_input_processor_driver_api scroll_acceleration_driver_api = {
    .handle_event = scroll_acceleration_handle_event,
};

#define SCROLL_ACCELERATION_INST(n)                                                    \
    static struct scroll_acceleration_data scroll_acceleration_data_##n = {};          \
    static const struct scroll_acceleration_config scroll_acceleration_config_##n = {  \
        .decay_ms = DT_INST_PROP_OR(n, decay_ms, 100),                                 \
        .max_acceleration = DT_INST_PROP_OR(n, max_acceleration, 10),                  \
        .min_speed_threshold = DT_INST_PROP_OR(n, min_speed_threshold, 1),             \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(n, scroll_acceleration_init, NULL,                           \
                          &scroll_acceleration_data_##n,                                \
                          &scroll_acceleration_config_##n, POST_KERNEL,                 \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                          \
                          &scroll_acceleration_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_ACCELERATION_INST)
