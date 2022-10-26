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


/* UUID Related */
#define PARG_UUID  BT_UUID_DECLARE_128(\
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,\
	0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

#define NRF_UUID  BT_UUID_DECLARE_128(\
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,\
	0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00)

uint16_t parg_uuid_vals[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
							0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00};

// tracking bool flags
bool bt_connected = false;
bool discover_complete = false;
bool write_complete = false;
bool read_complete = false;

// connection
static struct bt_conn *default_conn;

// data
int16_t accelsDataZ1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int16_t accelsDataZ2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static struct bt_uuid_128 nodesAccelZ1 = BT_UUID_INIT_128(
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
	0xa4, 0xd5, 0xe6, 0xf7, 0x28, 0x09, 0x00, 0x01);
static struct bt_uuid_128 nodesAccelZ2 = BT_UUID_INIT_128(
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
	0xa4, 0xd5, 0xe6, 0xf7, 0x28, 0x09, 0x00, 0x11);

void get_time_string_formatted(char* dest[64]) {

    int64_t time = k_uptime_get() / 1000;
    
    int64_t hours = time/60/60;
    char hours_msg[10];
    if (hours < 10)
        sprintf(hours_msg, "0%lld", hours);
    else
        sprintf(hours_msg, "%lld", hours);
    
    int64_t minutes = (time - hours * 60 * 60) / 60;
    char minutes_msg[10];
    if(minutes < 10)
        sprintf(minutes_msg, "0%lld", minutes);
    else
        sprintf(minutes_msg, "%lld", minutes);
    
    int64_t seconds =(time - hours * 60 * 60 - minutes * 60);
    char seconds_msg[10];
    if(seconds< 10)
        sprintf(seconds_msg, "0%lld", seconds);
    else
        sprintf(seconds_msg, "%lld", seconds);

    char time_msg[64];
    sprintf(time_msg, "%s:%s:%s", hours_msg, minutes_msg, seconds_msg);
    strcpy(dest, time_msg);
}

/* Basic Bluetooth Functions */

void bt_init(void) {
    int err;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
		return;
    }

}

/* void bt_adv(void) {
    int err;

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad_data, ARRAY_SIZE(ad_data), NULL, 0);
	if (err != 0) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

    printk("Advertising successfully started\n");
} */

static void connected(struct bt_conn *conn, uint8_t err){
	char addr[BT_ADDR_LE_STR_LEN];
	
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);
		bt_conn_unref(default_conn);
		default_conn = NULL;
		return;
	}

	default_conn = bt_conn_ref(conn);
	bt_connected = true;
	printk("Connected: %s \n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason){
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

/* Scanning Functions */

static bool parse_device(struct bt_data *data, void *user_data) {
	bt_addr_le_t *addr = user_data;
	int i;
	int matchedCount = 0;

	printk("[AD]: 0x%X data_len %u\n", data->type, data->data_len);

	char temp[20];
	if (data->type == BT_DATA_NAME_COMPLETE) {
		// here
		memcpy(temp, data->data, data->data_len);
		temp[data->data_len] = '\0';
		printk("Device Name: %s", temp);
	}

	if (data->type == BT_DATA_UUID128_ALL)
	{

		uint16_t temp = 0;
		for (i = 0; i < data->data_len; i++)
		{
			temp = data->data[i];
			if (temp == parg_uuid_vals[i])
			{
				matchedCount++;
			}
		}

		if (matchedCount == 16 && !(bt_connected)) // 16 bytes in scu_uuid_vals
		{
			//MOBILE UUID MATCHED
			printk("PARG Found, attempting to connect\n");

			int err = bt_le_scan_stop();
			k_msleep(10);

			if (err)
			{
				printk("Stop LE scan failed (err %d)\n", err);
				return true;
			}

			struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;

			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
									param, &default_conn);
			if (err)
			{
				printk("Create conn failed (err %d)\n", err);
				start_scan();
			}

			return false;
		}
	}
	return true;
}

// callback function of bluetooth scan
void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		struct net_buf_simple *ad) {
	// only consider connectable events
	if (type == BT_GAP_ADV_TYPE_ADV_IND ||
		type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
	{
		printk("Possible connectable device found.\n");
		bt_data_parse(ad, parse_device, (void *)addr);
		printk("%d\n", rssi);
	}
}

// scans for nearby devices
void start_scan() {
	int err;
	// scanning for base node
	struct bt_le_scan_param scanParams = {
	.type = BT_HCI_LE_SCAN_PASSIVE,
	.options = BT_LE_SCAN_OPT_NONE,
	.interval = 0x0500,
	.window = 0x0500};
	
	err = bt_le_scan_start(&scanParams, scan_cb);

	if (err) {
		printk("base node scan failed. %d\n", err);
	} else {
		printk("scanning for base node\n");
	}
}

uint8_t read_accel1(struct bt_conn *conn, uint8_t err,
                               struct bt_gatt_read_params *params,
                               const void *data, uint16_t length)
{

    memcpy(&accelsDataZ1, data, sizeof(accelsDataZ1));
    return 0;
}

uint8_t read_accel2(struct bt_conn *conn, uint8_t err,
                               struct bt_gatt_read_params *params,
                               const void *data, uint16_t length)
{

    memcpy(&accelsDataZ2, data, sizeof(accelsDataZ2));
    return 0;
}

void main() {
    int err;

    err = usb_enable(NULL);

    k_msleep(3000);
    if (err != 0) {
        printk("USB enable failed.\n");
    }

    bt_init();
    // bt_adv();
	bt_conn_cb_register(&conn_callbacks);

	start_scan();

	static struct bt_gatt_read_params read_params_accel1 = {
        .func = read_accel1,
        .handle_count = 0,
        .by_uuid.uuid = &nodesAccelZ1.uuid,
        .by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };
	static struct bt_gatt_read_params read_params_accel2 = {
        .func = read_accel2,
        .handle_count = 0,
        .by_uuid.uuid = &nodesAccelZ2.uuid,
        .by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };

	char time[64];

    while(1) {
		if (!bt_connected) {
			printk("bt not connected\n");
		} else {
			bt_gatt_read(default_conn, &read_params_accel1);
			bt_gatt_read(default_conn, &read_params_accel2);

			get_time_string_formatted(&time);
			printk("%s\n", time);
			// printk("%d\n", sizeof(accelsDataZ1));
			printk("Z1: ");
			for (int i = 0; i < sizeof(accelsDataZ1)/sizeof(accelsDataZ1[0]); i++) {
				printk("%d ", accelsDataZ1[i]);
			}
			printk("\n");

			printk("Z2: ");
			for (int i = 0; i < sizeof(accelsDataZ2)/sizeof(accelsDataZ2[0]); i++) {
				printk("%d ", accelsDataZ2[i]);
			}
			printk("\n");
			printk("\n");
		}
        k_msleep(10);
    }
}
