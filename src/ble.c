#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>
#include <logging/sys_log.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "ble.h"

static struct bt_uuid_128 uuid_bms = BT_UUID_INIT_128(
  0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
  0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 uuid_bms_values = BT_UUID_INIT_128(
  0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
  0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static ssize_t read_value(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, u16_t len, u16_t offset) {
  const uint8_t *value = attr->user_data;
  return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static struct bt_gatt_ccc_cfg vnd_ccc_cfg[BT_GATT_CCC_MAX];

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value) {
}

static struct bt_gatt_attr attrs_bms[] = {
  BT_GATT_PRIMARY_SERVICE(&uuid_bms),

  BT_GATT_CHARACTERISTIC(
    &uuid_bms_values.uuid,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY),

  BT_GATT_DESCRIPTOR(
    &uuid_bms_values.uuid,
    BT_GATT_PERM_READ,
    read_value,
    NULL,
    NULL),

  BT_GATT_CCC(vnd_ccc_cfg, vnd_ccc_cfg_changed),
};

static struct bt_gatt_service bms_svc = BT_GATT_SERVICE(attrs_bms);

static const struct bt_data ad[] = {
  BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))
};

static const struct bt_data sd[] = {
  BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BLUETOOTH_DEVICE_NAME, (sizeof(CONFIG_BLUETOOTH_DEVICE_NAME) - 1)),
};

static void connected(struct bt_conn *conn, u8_t err) {
  if (err) {
    printk("Connection failed (err %u)\n", err);
  } else {
    printk("Connected\n");
  }
}

static void disconnected(struct bt_conn *conn, u8_t reason) {
  printk("Disconnected (reason %u)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
  .connected = connected,
  .disconnected = disconnected,
};

void bt_ready(int err) {
  if (err) {
    SYS_LOG_ERR("Bluetooth enable failed: err=%d)", err);
    return;
  }

  SYS_LOG_INF("Bluetooth initialized");

  bt_gatt_service_register(&bms_svc);

  err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err) {
    SYS_LOG_ERR("Advertising failed to start (err %d)\n", err);
    return;
  }

  SYS_LOG_INF("Bluetooth advertising started");
}

void ble_init(void) {
  int err = bt_enable(bt_ready);

  if (err) {
    SYS_LOG_ERR("Bluetooth enable failed: err=%d)", err);
    return;
  }

  bt_conn_cb_register(&conn_callbacks);
}

void ble_notify(uint8_t *data, int length) {
  bt_gatt_notify(NULL, &attrs_bms[2], data, length);
}
