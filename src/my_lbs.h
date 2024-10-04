
#ifndef BT_LBS_H_
#define BT_LBS_H_

/**@file
 * @defgroup bt_lbs LED Button Service API
 * @{
 * @brief API for the LED Button Service (LBS).
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/** @brief LBS Service UUID. */
#define BT_UUID_LBS_VAL \
	BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/* Assign a UUID to the SENSOR characteristic */
#define BT_UUID_LBS_MYSENSOR_VAL3 \
	BT_UUID_128_ENCODE(0x1a5a5612, 0x8cc7, 0x4c88, 0xb210, 0xd697b3a75e23)

/* Convert the array to a generic UUID */
#define BT_UUID_BINAHKI1       		BT_UUID_DECLARE_128(BT_UUID_LBS_VAL)

#define BT_UUID_LBS_MYSENSOR3	    BT_UUID_DECLARE_128(BT_UUID_LBS_MYSENSOR_VAL3)

/** @brief Callback type for when an LED state change is received. */
typedef void (*led_cb_t)(const bool led_state);

/** @brief Callback struct used by the LBS Service. */
struct my_lbs_cb {
	/** LED state change callback. */
	led_cb_t    led_cb;
};

/** @brief Initialize the BINAHKI1 Service.
 *
 * This function registers application callback functions with the My LBS
 * Service 
 *
 * @param[in] callbacks Struct containing pointers to callback functions
 *			used by the service. This pointer can be NULL
 *			if no callback functions are defined.
 *
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int binahki1_init();

/** @brief Send the sensor value as notification.
 *
 * Essa função envia a string por bluetooth
 *
 * @param[in] sensor3_value Dado simulado.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
char sensors_notify(char* sensor3_value);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_LBS_H_ */
