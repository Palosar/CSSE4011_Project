// zephyr
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <usb/usb_device.h>

// bluetooth
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>


#define PARG_UUID  BT_UUID_DECLARE_128(\
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,\
	0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

#define NRF_UUID  BT_UUID_DECLARE_128(\
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,\
	0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

// bt connection
static struct bt_conn *default_conn;

// tracking bool flags
bool bt_connected = false;
bool discover_complete = false;
bool write_complete = false;
bool read_complete = false;

// 16 accelerometers data
int16_t accelsDataZ1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int16_t accelsDataZ2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static struct bt_uuid_128 nodesAccelZ1 = BT_UUID_INIT_128(
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
	0xa4, 0xd5, 0xe6, 0xf7, 0x28, 0x09, 0x00, 0x01);
static struct bt_uuid_128 nodesAccelZ2 = BT_UUID_INIT_128(
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
	0xa4, 0xd5, 0xe6, 0xf7, 0x28, 0x09, 0x00, 0x11);


/* Basic Bluetooth Functions */

void bt_init(void) {
    int err;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
		return;
    }

}

static const struct bt_data ad_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
            0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
            0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00),
};

void bt_adv(void) {
    int err;

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad_data, ARRAY_SIZE(ad_data), NULL, 0);
	if (err != 0) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

    printk("Advertising successfully started\n");
}


/* BT Service Related */

// BT gatt read characteristics
static ssize_t read_accelsz1(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset) {
    const int16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(accelsDataZ1));
}

static ssize_t read_accelz2(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset) {
    const int16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(accelsDataZ2));
}

// BT service
BT_GATT_SERVICE_DEFINE(parg_svc,
	BT_GATT_PRIMARY_SERVICE(PARG_UUID),
	BT_GATT_CHARACTERISTIC(&nodesAccelZ1.uuid,
			BT_GATT_CHRC_READ,
			BT_GATT_PERM_READ,
			read_accelsz1, NULL, &accelsDataZ1),
	BT_GATT_CHARACTERISTIC(&nodesAccelZ2.uuid,
			BT_GATT_CHRC_READ,
			BT_GATT_PERM_READ,
			read_accelz2, NULL, &accelsDataZ2)
);


/* BT Connection Related */

static void connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];
	
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        printk("Failed to connect to %s (%u)\n", addr, err);
        bt_conn_unref(default_conn);
        default_conn = NULL;
        return;
    }

	bt_connected = true;
	discover_complete = false;;
    printk("Connected to %s\n", addr);
	
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

    bt_conn_unref(default_conn);
    default_conn = NULL;
    bt_connected = false;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

void main() {
    int err;

    err = usb_enable(NULL);

    k_msleep(3000);
    if (err != 0) {
        printk("USB enable failed.\n");
    }

    bt_init();
    bt_conn_cb_register(&conn_callbacks);
    bt_adv();

    for (int i = 0; i < 8; i++) {
        accelsDataZ1[i] += 900;
        accelsDataZ2[i] += 900;
    }

    while(1) {
		if (!bt_connected) {
			printk("bt not connected\n");
		} else {
			printk("bt connected\n");
		}
        for (int i = 0; i < 8; i++) {
            accelsDataZ1[i] += 5;
            accelsDataZ2[i] += 5;
        }
        k_msleep(1000);
    }
}
