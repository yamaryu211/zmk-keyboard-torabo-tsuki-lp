// SPDX-License-Identifier: GPL-2.0-or-later

#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <zmk/split/bluetooth/uuid.h>

LOG_MODULE_REGISTER(split_peripheral_adv_recovery, CONFIG_ZMK_LOG_LEVEL);

static atomic_t split_connected;

static const struct bt_data split_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME, 0x0f, 0x18),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, ZMK_SPLIT_BT_SERVICE_UUID),
};

static void copy_bonded_central(const struct bt_bond_info *info, void *user_data) {
    bt_addr_le_t *central = user_data;

    if (bt_addr_le_cmp(&info->addr, BT_ADDR_LE_NONE) != 0) {
        bt_addr_le_copy(central, &info->addr);
    }
}

static int start_recovery_advertising(void) {
    bt_addr_le_t central = bt_addr_le_none;

    bt_foreach_bond(BT_ID_DEFAULT, copy_bonded_central, &central);

    if (bt_addr_le_cmp(&central, BT_ADDR_LE_NONE) != 0) {
        const struct bt_le_adv_param param = *BT_LE_ADV_CONN_DIR_LOW_DUTY(&central);

        return bt_le_adv_start(&param, NULL, 0, NULL, 0);
    }

    return bt_le_adv_start(BT_LE_ADV_CONN, split_ad, ARRAY_SIZE(split_ad), NULL, 0);
}

static void advertising_recovery_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(advertising_recovery_work, advertising_recovery_handler);

static void schedule_recovery(k_timeout_t delay) {
    if (!atomic_get(&split_connected)) {
        k_work_reschedule(&advertising_recovery_work, delay);
    }
}

static void advertising_recovery_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (atomic_get(&split_connected)) {
        return;
    }

    const int err = start_recovery_advertising();

    if (err == 0) {
        LOG_INF("Recovered split peripheral advertising");
    } else if (err == -EALREADY) {
        LOG_DBG("Split peripheral advertising is already active");
    } else {
        LOG_WRN("Failed to recover split peripheral advertising (%d)", err);
    }

    schedule_recovery(K_MSEC(CONFIG_TORABO_SPLIT_PERIPHERAL_ADV_RECOVERY_INTERVAL_MS));
}

static void recovery_connected(struct bt_conn *conn, uint8_t err) {
    ARG_UNUSED(conn);

    if (err != 0) {
        schedule_recovery(
            K_MSEC(CONFIG_TORABO_SPLIT_PERIPHERAL_ADV_RECOVERY_INITIAL_DELAY_MS));
        return;
    }

    atomic_set(&split_connected, 1);
    k_work_cancel_delayable(&advertising_recovery_work);
    LOG_INF("Split central connected; advertising recovery stopped");
}

static void recovery_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(conn);

    atomic_clear(&split_connected);
    LOG_INF("Split central disconnected (reason 0x%02x); scheduling advertising recovery", reason);
    schedule_recovery(K_MSEC(CONFIG_TORABO_SPLIT_PERIPHERAL_ADV_RECOVERY_INITIAL_DELAY_MS));
}

static struct bt_conn_cb recovery_conn_callbacks = {
    .connected = recovery_connected,
    .disconnected = recovery_disconnected,
};

static void find_existing_connection(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);

    atomic_set(&split_connected, 1);
}

static int split_peripheral_adv_recovery_init(void) {
    bt_conn_cb_register(&recovery_conn_callbacks);
    bt_conn_foreach(BT_CONN_TYPE_LE, find_existing_connection, NULL);

    schedule_recovery(K_MSEC(CONFIG_TORABO_SPLIT_PERIPHERAL_ADV_RECOVERY_INITIAL_DELAY_MS));
    return 0;
}

SYS_INIT(split_peripheral_adv_recovery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
