/*
 * SPDX-License-Identifier: MIT
 *
 * Inertia scrolling input processor for REL_X/REL_Y devices (trackpad without scroller-mode).
 * Tracks XY velocity while finger is moving, then injects decaying XY events after finger lift.
 * Injected events flow through the full listener chain (including zip_xy_to_scroll_mapper).
 *
 * Place this processor FIRST in each sub-node so it sees raw XY events before conversion.
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_inertia

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define INERTIA_TICK_MS 10
#define Q8_SHIFT        8
#define Q8_ONE          256

struct scroll_inertia_config {
    const struct device *device;
    int32_t decay_permille;
    int32_t stop_threshold_q8;
    int32_t idle_timeout_ms;
};

struct scroll_inertia_data {
    const struct device *proc_dev;
    struct k_work_delayable work;

    int32_t vel_x_q8;
    int32_t vel_y_q8;
    int32_t rem_x_q8;
    int32_t rem_y_q8;
    int64_t last_x_time_ms;
    int64_t last_y_time_ms;

    bool inertia_active;
    bool injecting;
};

static int32_t inertia_abs(int32_t v) { return v < 0 ? -v : v; }

static void update_velocity(int32_t *vel_q8, int64_t *last_time_ms, int16_t delta, int64_t now_ms) {
    if (delta == 0) {
        return;
    }

    int32_t sample_q8;
    if (*last_time_ms <= 0 || now_ms <= *last_time_ms) {
        sample_q8 = (int32_t)delta << Q8_SHIFT;
    } else {
        int64_t dt_ms = now_ms - *last_time_ms;
        if (dt_ms < 1) {
            dt_ms = 1;
        }
        sample_q8 = ((int32_t)delta << Q8_SHIFT) * INERTIA_TICK_MS / (int32_t)dt_ms;
    }

    /* Moving-average filter to smooth abrupt speed changes */
    *vel_q8 = (*vel_q8 * 3 + sample_q8) / 4;
    *last_time_ms = now_ms;
}

static void inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_inertia_data *data =
        CONTAINER_OF(dwork, struct scroll_inertia_data, work);
    const struct scroll_inertia_config *cfg = data->proc_dev->config;

    /* First fire after idle timeout: check if there's enough velocity to coast */
    if (!data->inertia_active) {
        bool x_ok = inertia_abs(data->vel_x_q8) >= cfg->stop_threshold_q8;
        bool y_ok = inertia_abs(data->vel_y_q8) >= cfg->stop_threshold_q8;
        if (!x_ok && !y_ok) {
            return;
        }
        data->inertia_active = true;
        data->rem_x_q8 = 0;
        data->rem_y_q8 = 0;
    }

    bool x_active = inertia_abs(data->vel_x_q8) >= cfg->stop_threshold_q8;
    bool y_active = inertia_abs(data->vel_y_q8) >= cfg->stop_threshold_q8;

    if (!x_active) {
        data->vel_x_q8 = 0;
        data->rem_x_q8 = 0;
    }
    if (!y_active) {
        data->vel_y_q8 = 0;
        data->rem_y_q8 = 0;
    }

    if (!x_active && !y_active) {
        data->inertia_active = false;
        return;
    }

    data->rem_x_q8 += data->vel_x_q8;
    int32_t x_delta = data->rem_x_q8 / Q8_ONE;
    data->rem_x_q8 -= x_delta * Q8_ONE;

    data->rem_y_q8 += data->vel_y_q8;
    int32_t y_delta = data->rem_y_q8 / Q8_ONE;
    data->rem_y_q8 -= y_delta * Q8_ONE;

    /* Inject XY events — they flow through the full listener chain including zip_xy_to_scroll_mapper.
     * The injecting flag prevents this processor from updating velocity for its own injected events. */
    data->injecting = true;
    if (x_delta != 0) {
        input_report_rel(cfg->device, INPUT_REL_X, x_delta, (y_delta == 0), K_FOREVER);
    }
    if (y_delta != 0) {
        input_report_rel(cfg->device, INPUT_REL_Y, y_delta, true, K_FOREVER);
    }
    data->injecting = false;

    data->vel_x_q8 = (data->vel_x_q8 * cfg->decay_permille) / 1000;
    data->vel_y_q8 = (data->vel_y_q8 * cfg->decay_permille) / 1000;

    k_work_schedule(&data->work, K_MSEC(INERTIA_TICK_MS));
}

static int scroll_inertia_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    if (event->code != INPUT_REL_X && event->code != INPUT_REL_Y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    struct scroll_inertia_data *data = dev->data;
    const struct scroll_inertia_config *cfg = dev->config;

    /* Inertia-injected events pass through without touching state */
    if (data->injecting) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Real event: stop any running inertia and restart the idle timer */
    data->inertia_active = false;

    int64_t now_ms = k_uptime_get();
    if (event->code == INPUT_REL_X) {
        update_velocity(&data->vel_x_q8, &data->last_x_time_ms, (int16_t)event->value, now_ms);
    } else {
        update_velocity(&data->vel_y_q8, &data->last_y_time_ms, (int16_t)event->value, now_ms);
    }

    /* k_work_reschedule cancels any pending tick and sets a new deadline */
    k_work_reschedule(&data->work, K_MSEC(cfg->idle_timeout_ms));

    return ZMK_INPUT_PROC_CONTINUE;
}

static int scroll_inertia_init(const struct device *dev) {
    struct scroll_inertia_data *data = dev->data;
    data->proc_dev = dev;
    k_work_init_delayable(&data->work, inertia_work_handler);
    data->vel_x_q8 = 0;
    data->vel_y_q8 = 0;
    data->rem_x_q8 = 0;
    data->rem_y_q8 = 0;
    data->last_x_time_ms = 0;
    data->last_y_time_ms = 0;
    data->inertia_active = false;
    data->injecting = false;
    return 0;
}

static const struct zmk_input_processor_driver_api scroll_inertia_driver_api = {
    .handle_event = scroll_inertia_handle_event,
};

#define SCROLL_INERTIA_INST(n)                                                               \
    static struct scroll_inertia_data scroll_inertia_data_##n = {};                          \
    static const struct scroll_inertia_config scroll_inertia_config_##n = {                  \
        .device            = DEVICE_DT_GET(DT_INST_PHANDLE(n, device)),                      \
        .decay_permille    = DT_INST_PROP_OR(n, decay_permille, 920),                        \
        .stop_threshold_q8 = DT_INST_PROP_OR(n, stop_threshold_q8, 96),                     \
        .idle_timeout_ms   = DT_INST_PROP_OR(n, idle_timeout_ms, 50),                       \
    };                                                                                       \
    DEVICE_DT_INST_DEFINE(n, scroll_inertia_init, NULL,                                      \
                          &scroll_inertia_data_##n,                                          \
                          &scroll_inertia_config_##n, POST_KERNEL,                           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                               \
                          &scroll_inertia_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_INERTIA_INST)
