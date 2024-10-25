
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "my_lbs.h"

char* dado_string;
char  dado[] = "Presenca: 1, Bateria: 3,6";

static ssize_t read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);

LOG_MODULE_DECLARE(HardwareTCC);

/* Declaração do serviço bluetooth tcc */
BT_GATT_SERVICE_DEFINE(my_tcc_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_TCC),

	/* Create and add the SENSOR characteristic and its CCCD  */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_MYSENSOR3,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_callback, NULL,
			       &dado_string),

	BT_GATT_CCC(NULL,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Função de inicialização da característica TCC */
int tcc_init()
{
	printk("Inicialização da caracteristica TCC\n");
	return 0;
}

/* Função de envio dos dados por meio de notificação bluetooth */
char sensors_notify(char* sensor3_value)		
{
	dado_string = sensor3_value;
	return bt_gatt_notify(NULL, &my_tcc_svc.attrs[2], sensor3_value, strlen(sensor3_value));
}

static ssize_t read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{

	// return bt_gatt_attr_read(conn, &my_tcc_svc.attrs[2], buf, strlen(*dado_string), offset, dado_string, strlen(*dado_string));
	return bt_gatt_attr_read(conn, &my_tcc_svc.attrs[2], buf, 26, offset, dado_string, 26);
}