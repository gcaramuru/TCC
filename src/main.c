
#include <zephyr/kernel.h>

/* Bibliotecas BLE */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include "my_lbs.h"

/* Bibliotaca LED */
#include <dk_buttons_and_leds.h>

/* Biliotecas GPIO */
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/gpio.h>

// #include <stdio.h>

/* Bibliotecas de gerenciamento de energia */
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
	
/* Bibliotecas de dipositivos e I2C */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

/* Bibliotecas dos sensores e auxiliares */
#include "max30102.c"
#include "bq25180.c"

/* Biblioteca de tempo */
#include <sys/time.h>

/* Biblioteca entrada analógica */
#include <zephyr/drivers/adc.h>

/* Biblioteca para reinicializar o código */
// #include <zephyr/sys/reboot.h>

/* Definição do nó do LED */
#define LED5_NODE DT_ALIAS(led5)

/* Struct que pega o nó do LED */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED5_NODE, gpios);

/* Definição do nó analógico */
#define ADC_NODE DT_NODELABEL(adc)

/* Struct de configuração do canal analógico */
const struct adc_channel_cfg ch0_cfg_dt =
     ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_0));

/* Definição do divisor de tensão, para cálculo do valor da bateria */
#define VOLTAGE_DIVISOR_RATIO (100.0 / (100.0 + 100.0)) // R2 / (R1 + R2)

/* Struct que aponta para o nó analógico */
const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

/* Inicialização do buff analógico */
static int16_t adc_buffer;

/* Struct com configurações do canal analógico */
struct adc_sequence sequence = {
        .channels    = BIT(0),
        .buffer      = &adc_buffer,
        .buffer_size = sizeof(adc_buffer),
        .resolution  = 12,
    };

/* Struct com configurações de parâmetros de publicidade BLE */
static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM((BT_LE_ADV_OPT_CONNECTABLE|BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
                800, /*Min Advertising Interval 500ms (800*0.625ms) */
                801, /*Max Advertising Interval 500.625ms (801*0.625ms)*/
                NULL); /* Set to NULL for undirected advertising*/

/* Inicialização de struct e função relacionada a mudanção no tamanho do MTU */
static struct bt_gatt_exchange_params exchange_params; 
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);

/* Definições do nome e comprimento do BLE */
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

/* Definições necessárias para utilização das threads */
#define STACKSIZE 1024
#define PRIORITY 7

/* Definição do nó I2C */
#define I2C_NODE DT_NODELABEL(i2c1)


/* Declaração e inicialização de variáveis */
#define BUFFER_MAX 240												// Valor máximo do buffer que auxilia na transformação dos valores em string
#define RED_LED 0X4B												// Variável que controla a intensidade do led vermelho
#define IR_LED 0x3C													// Variável que controla a intensidade do led infravermelho
#define LIMIAR_DETEC 50000											// Variável que define o limiar para considerar uma detecção

bool  status			= 0;										// Variável que indica o status de conexão BLE
int   presenca			= 0;		                                // Variável que indica a presença
float temperatura		= 0.00;										// Variável que armazena a leitura da temperatura
float bateria	 		= 0.00;										// Variável que armazena a leitura da bateria
char  json_str[]		= "inicialização da string dos sensores";	// Variável da mensagem dos dados que são enviados por bluetooth
char  pre[]				= "inicialização da string de presença";	// Variável que auxilia na montagem da variável a ser enviada (json_str)
char  bat[]				= "inicialização da string da bateria";		// Variável que auxilia na montagem da variável a ser enviada (json_str2)

uint32_t tempo_inicial;												// Variável auxiliar no controle de tempo
uint32_t tempo_atual;												// Variável auxiliar no controle de tempo

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

/* Definições necessárias para utilização do controlador de carga */
#define BQ25180_TEMP_HIGH_REG 0x00
#define BQ25180_TEMP_LOW_REG 0x02
#define BQ25180_CONFIG_REG 0x03
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MIN_BAT_REG_mV 3500U
#define MAX_BAT_REG_mV 4200U
#define MIN_BAT_UNDERVOLTAGE_mV 2000U
#define MAX_BAT_UNDERVOLTAGE_mV 3000U
#define MIN_IN_CURR_mA 5U
#define MAX_IN_CURR_mA 1000U
#define BQ25180_DEVICE_ADDRESS 0x6A

/* Função com as chamadas de configuração do controlador de carga */
void bq25180_setup()
{
	// Configuração inicial do bq25180
	// bq25180_reset(true);
	bq25180_enable_push_button(true);
	bq25180_set_fastcharge_current(1000);								// Configura para 1000mA (1C)
	bq25180_set_precharge_current(true); 								// Define para 10% da corrente de carga máxima
	bq25180_enable_thermal_protection(false);
	bq25180_set_precharge_threshold(2800);							   	// Define para 2800mV
	bq25180_set_battery_discharge_current(BQ25180_BAT_DISCHAGE_500mA); 	// Configura para 500mA (ou outra opção adequada)
	bq25180_set_battery_under_voltage(3200);						   	// Configura para 3200mV (ou outra opção adequada)
	bq25180_enable_battery_charging(true);

}

/* Função com retorno do tempo em ms*/
uint32_t tempo_em_ms(void) {
    return k_uptime_get_32();
}

/* Função de coleta dos dados */
static void data_colection(const struct device *dev_i2c)
{
	/* Leitura da bateria */
	adc_read(adc_dev, &sequence);
    float adc_voltage = ((float)adc_buffer / (1 << 12)) * 0.6 * 6;
    bateria = adc_voltage / VOLTAGE_DIVISOR_RATIO;

	/* Leitura do sensor de batimento cardíaco e oximetria */
	setPulseAmplitudeRed(RED_LED);
    setPulseAmplitudeIR(IR_LED);
	k_msleep(200); 

	if(max30102_GetIR() > LIMIAR_DETEC && max30102_GetRed() > LIMIAR_DETEC){
		presenca = 1;
	
		setPulseAmplitudeRed(0x00);
		setPulseAmplitudeIR(0x00);
		// pm_device_action_run(dev_i2c, PM_DEVICE_ACTION_TURN_OFF);
		// k_msleep(14000);
		// pm_device_action_run(dev_i2c, PM_DEVICE_ACTION_TURN_ON);
	}
	else{
		presenca = 0;
        setPulseAmplitudeRed(0x00);
		setPulseAmplitudeIR(0x00);
		// pm_device_action_run(dev_i2c, PM_DEVICE_ACTION_TURN_OFF);
		// k_msleep(59000);
		// pm_device_action_run(dev_i2c, PM_DEVICE_ACTION_TURN_ON);
	}

	/* Converção dos valores para string */
	sprintf(bat, "%.2f", bateria);
	sprintf(pre, "%d", presenca);

	/* Construção da string que será enviada via BLE, no serviço BINAHKI1*/
	snprintf(json_str, BUFFER_MAX, "{\"Presence\": %s, \"Battery\": %s}", pre, bat);		
}

/* Função de chamada da thread responsável pelo envio de dados via BLE */
void send_data_thread(void)
{	
	const struct device *dev_i2c;
	dev_i2c = DEVICE_DT_GET(I2C_NODE);

	while(1){

		while(status == 1){
			tempo_inicial = tempo_em_ms();
			tempo_atual = tempo_em_ms();

			/* Chamada da função de coleta de dados */
			data_colection(dev_i2c);
			
			/* Chamada das funções que fazem o envio dos dados via bluetooth */
			sensors_notify(json_str);

			tempo_atual = tempo_em_ms();
			k_msleep(60000 - (tempo_atual - tempo_inicial));
		}
		
		while (status == 0)
		{
			k_msleep(5000);
		}
	}	
}

/* Função de configuração de tamanho do MTU */
static void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    // if (err) {
        // printk("data_len_update failed (err %d)", err);
    // }
}

/* Função de configuração de tamanho do MTU */
static void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    // if (err) {
        // printk("bt_gatt_exchange_mtu failed (err %d)", err);
    // }
}

/* Função de callback para conexão BLE */
static void on_connected(struct bt_conn *conn, uint8_t err)
{
	const struct device *dev_i2c;
	dev_i2c = DEVICE_DT_GET(I2C_NODE);

	if (err) {
		// printk("Falha de conexão (err %u)\n", err);
		return;
	}
	// printk("Conectado\n");
	status = 1;
	/* Chamada de funções para configuração de tamanho do MTU */
	update_data_length(conn);
	update_mtu(conn);

	gpio_pin_set_dt(&led, 1);
	pm_device_action_run(dev_i2c, PM_DEVICE_ACTION_TURN_ON);
}

/* Função de callback para desconexão do BLE */
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	const struct device *dev_i2c;
	dev_i2c = DEVICE_DT_GET(I2C_NODE);

	// printk("Desconectado (reason %u)\n", reason);
	status = 0;
	gpio_pin_set_dt(&led, 0);
	setPulseAmplitudeRed(0x00);
	setPulseAmplitudeIR(0x00);
	pm_device_action_run(dev_i2c, PM_DEVICE_ACTION_TURN_OFF);
}

/* Função de callback para alteração de tamanho do MTU */
void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    uint16_t tx_len     = info->tx_max_len; 
    uint16_t tx_time    = info->tx_max_time;
    uint16_t rx_len     = info->rx_max_len;
    uint16_t rx_time    = info->rx_max_time;
    // printk("Data length updated. Length %d/%d bytes, time %d/%d us\n", tx_len, rx_len, tx_time, rx_time);	
}

/* Struct de callbacks BLE */
struct bt_conn_cb connection_callbacks = {
    .connected              = on_connected,
    .disconnected           = on_disconnected,  
	.le_data_len_updated    = on_le_data_len_updated,
};

/* Função para alteração de tamanho do MTU */
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params)
{
	// printk("MTU exchange %s\n", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
        // printk("Novo MTU: %d bytes\n", payload_mtu);
    }
}

/* Função de inicialização do dispositivo, realiza as configurações básicas */
void begining_thread(void)
{
	int err;
    
	const struct device *dev_i2c;
	dev_i2c = DEVICE_DT_GET(I2C_NODE);

	// printk("Hardware Binahki\n");
	
	/* Chamada da função de setup do controlador de carga */
	bq25180_setup();

	/* Chamada da função de inicialização do sensor MAX30102 */
	err = max30102_Begin();
	// if(err != 0){
	// 	printk("Reeboting system\n");
	// 	sys_reboot(SYS_REBOOT_COLD);
	// }

	/* Inicialização da entrada analógica */
    if (!device_is_ready(adc_dev)) {
        // printk("ADC não pronto\n");
        return;
    }

	/* Configuração da entrada analógica */
    err = adc_channel_setup(adc_dev, &ch0_cfg_dt);
    if (err) {
        // printk("Erro na configuração da entrada analógica: %d\n", err);
        return;
    }

	/* Inicialização do led */
	err = dk_leds_init();
	if (err) {
		// printk("Falha na inicialização dos LEDs (err %d)\n", err);
		return;
	}
	
	/* Inicialização do BLE */
	err = bt_enable(NULL);
	if (err) {
		// printk("Falha na inicialização do Bluetooth (err %d)\n", err);
		return;
	}

	/* Registro de callbacks do BLE */
    bt_conn_cb_register(&connection_callbacks);

	/* Inicialização da característica BINAHKI1 */
	err = binahki1_init();
	if (err) {
		// printk("Falha na inicialização da característica BINAHKI1 (err:%d)\n", err);
		return;
	}

	// printk("Bluetooth inicializado\n");

	/* Inicialazação da publicidade BLE */
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		// printk("Falha no começo da publicidade (err %d)\n", err);
		return;
	}
	
	// printk("Publicidade inicializada\n");

	gpio_pin_set_dt(&led, 0);
	setPulseAmplitudeRed(0x00);
	setPulseAmplitudeIR(0x00);
}

/* Thread de inicialização do dispositivo */
K_THREAD_DEFINE(begining_thread_id, STACKSIZE, begining_thread, NULL, NULL, NULL, PRIORITY, 0, 0);
/* Thread de coleta e envio dos dados (delay de 100ms para garantir a inicialização do dispositivo) */
K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread, NULL, NULL, NULL, PRIORITY, 0, 100);