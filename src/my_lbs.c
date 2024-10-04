
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

LOG_MODULE_DECLARE(HardwareBinahki);

/* Declaração do serviço bluetooth binahki1 */
BT_GATT_SERVICE_DEFINE(my_binahki1_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_BINAHKI1),

	/* Create and add the SENSOR characteristic and its CCCD  */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_MYSENSOR3,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL,
			       NULL),

	BT_GATT_CCC(NULL,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Função de inicialização da característica BINAHKI1 */
int binahki1_init()
{
	return 0;
}

/* Função de envio dos dados por meio de notificação bluetooth */
char sensors_notify(char* sensor3_value)		
{
	return bt_gatt_notify(NULL, &my_binahki1_svc.attrs[2], sensor3_value, strlen(sensor3_value));
}