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

// data related
static struct bt_uuid_128 accelDataUuid = BT_UUID_INIT_128(
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
	0xa4, 0xd5, 0xe6, 0xf7, 0x28, 0x09, 0x00, 0x03);

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
}AccelData;

#define MAX_NUM_ACCELEROMETERS 16

static AccelData data_accels[MAX_NUM_ACCELEROMETERS]

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


// BT service
BT_GATT_SERVICE_DEFINE(parg_svc,
	BT_GATT_PRIMARY_SERVICE(PARG_UUID),
	// BT_GATT_CHARACTERISTIC(&scu_pkt_uuid.uuid,
	// 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
	// 		       BT_GATT_PERM_READ |
	// 		       BT_GATT_PERM_WRITE,
	// 		       NULL, NULL, NULL),	
	BT_GATT_CCC(NULL,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void send_accel_data() {
    bt_gatt_notify(default_conn, &parg_svc.attrs[1], dataAccels, sizeof(dataAccels));
}

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

    for (int i = 0; i < MAX_NUM_ACCELEROMETERS; i++) {
        AccelData temp;
        temp.x = 0;
        temp.y = 0;
        temp.z = 0;

        data_accels[i] = temp;
    }

    while(1) {
		if (!bt_connected) {
			printk("bt not connected\n");
		} else {
			printk("bt connected\n");
            send_accel_data();
		}
        k_msleep(1000);
    }
}