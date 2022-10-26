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

static struct bt_uuid_128 accelDataUuid = BT_UUID_INIT_128(
	0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x02, 0x21,
	0xa4, 0xd5, 0xe6, 0xf7, 0x28, 0x09, 0x00, 0x03);

// tracking bool flags
bool bt_connected = false;
bool discover_complete = false;
bool write_complete = false;
bool read_complete = false;

// connection
static struct bt_conn *default_conn;

// Accelerometer Data
#define MAX_NUM_ACCELEROMETERS 16

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
}AccelData;

static AccelData data_accels[MAX_NUM_ACCELEROMETERS];

int count = 0;

// notify function related
static struct bt_gatt_discover_params discoverParams;
static struct bt_gatt_subscribe_params subscribeParams;

// all functions
void get_time_string_formatted(char* dest[64]);
void bt_init(void);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static bool parse_device(struct bt_data *data, void *user_data);
void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		struct net_buf_simple *ad);
uint8_t discover_primary(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params);
static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params);
static void gatt_discover(void);

// Returns the current time of parg
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


	gatt_discover();

	return;
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


static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params)
{
	int err;
	count++;
	// reached the end of service
	if (attr == NULL) {
		printk("reached end of service\n");

		discover_complete = true;

		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	return BT_GATT_ITER_CONTINUE;
}

static void gatt_discover(void)
{
	static struct bt_gatt_discover_params discover_params;
	int err;

	printk("Discovering services and characteristics\n");

	discover_params.uuid = PARG_UUID;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(default_conn, &discover_params);
	if (err != 0) {
		printk("Discover failed(err %d)\n", err);
	}

	while(!discover_complete) {
		k_msleep(1);
	};

	printk("Discover complete\n");
}


// uint8_t discover_primary(struct bt_conn *conn,
//                          const struct bt_gatt_attr *attr,
//                          struct bt_gatt_discover_params *params) {
//     if (!attr) {
//         (void)memset(params, 0, sizeof(*params));
//         return BT_GATT_ITER_STOP;
//     }

// 	subscribeParams.value = BT_GATT_CCC_INDICATE;
// 	subscribeParams.ccc_handle = attr->handle;
// 	subscribeParams.notify = notify_func;
// 	subscribeParams.value_handle = scuPacketValHandle;
// 	int err = bt_gatt_subscribe(conn, &subscribeParams);
// 	if (err) {
// 		sprintf(tempMsg, "Discover failed(err %d)\n", err);
// 		ahu_ble_log_message(AHU_BLE_MSG_DBG, tempMsg);
// 	} else {
// 		sprintf(tempMsg, "Device discovery successful");
// 		ahu_ble_log_message(AHU_BLE_MSG_DBG, tempMsg);
// 	}

//     return BT_GATT_ITER_STOP;    
// }

// uint8_t notify_func(struct bt_conn *conn,
//                     struct bt_gatt_subscribe_params *params,
//                     const void *data, uint16_t length) {
//     if (!data) {
//         params->value_handle = 0U;
//         return BT_GATT_ITER_STOP;
//     }
//     uint8_t *dataPacket = (uint8_t *)data;
    
//     // Read the pre-amble
//     uint8_t preAmble = dataPacket[0];
    
//     if (preAmble != 170) {
//         sprintf(tempMsg, "Invalid packet");
//         ahu_ble_log_message(AHU_BLE_MSG_DBG, tempMsg);
//         return -1;
//     }
    
//     // Read the message type, must be a response
//     uint8_t type = dataPacket[1] >> 4;
//     if (type != 2) {
//         sprintf(tempMsg, "Invalid response");
//         ahu_ble_log_message(AHU_BLE_MSG_DBG, tempMsg);
//         return -1;
//     }

//     uint8_t dataLength = dataPacket[1] & 0xF;
    
// #ifdef CONFIG_PRAC2_CODE       
//     uint8_t deviceID = dataPacket[2];
//     uint8_t devType = dataPacket[3];
//     if (devType == 2) {    // Beacon
//         uint8_t rssiVal = dataPacket[4];
//         update_device_data(deviceID, devType, rssiVal, 0);
//     } 
//     else if (devType == 1) {   // Ultrasonic Sensor
//         uint8_t mantissa[3];
//         for(int j = 0; j < 3; j++) {
//             mantissa[j] = dataPacket[j + 4];
//         }
        
//         double sensorValue = (double)sys_get_le24(mantissa);
//         update_device_data(deviceID, devType, 0, sensorValue);
//     }
//     else if (devType == 3) {   // IMU data
//         uint8_t mantissa[3];
//         for(int j = 0; j < 3; j++) {
//             mantissa[j] = dataPacket[j + 4];
//         }
        
//         double sensorValue = (double)sys_get_le24(mantissa);
//         update_device_data(deviceID, devType, 0, sensorValue);
//     }
    
//     return BT_GATT_ITER_CONTINUE;
    
// #endif


void main() {
    int err;

    err = usb_enable(NULL);

    k_msleep(3000);
    if (err != 0) {
        printk("USB enable failed.\n");
    }
	count = 0;

    bt_init();
    // bt_adv();
	bt_conn_cb_register(&conn_callbacks);

	start_scan();

	char time[64];

    while(1) {
		if (!bt_connected) {
			printk("bt not connected\n");
		} else {
			printk("bt connected\n");
			if (discover_complete) {
				printk("%d\n", count);
			} else {

			}
		}
        k_msleep(500);
    }
}
