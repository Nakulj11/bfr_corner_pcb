#define PIN_SERIAL1_TX 1
#define RS485_DEFAULT_DE_PIN A6
#define RS485_DEFAULT_RE_PIN A5


#include <ArduinoRS485.h>

#define BOARD_INDEX 0
#define TX_GPIO_NUM   21  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX

#define CAN_STR_GAG_ID_1 0x01
#define CAN_STR_GAG_ID_2 0x02
#define CAN_STR_GAG_ID_3 0x03
#define STR_GAG_PIN_1 A0
#define STR_GAG_PIN_2 A1
#define STR_GAG_PIN_3 A2
#define STR_GAG_READ_INT 10000

#define CAN_SHOCK_POT_ID 0x04
#define SHOCK_POT_PIN A1
#define SHOCK_POT_READ_INT 10000

#define WHEEL_SPEED_PIN 1
#define WHEEL_SPEED_ID 0x05
#define WHEEL_SPEED_READ_INT 10000

#define BRAKE_TEMP_READ_INT 10000
#define BRAKE_TEMP_ID 0x06

#define WHEEL_TEMP_READ_INT 10000
#define WHEEL_TEMP_ID_START 0x07

uint8_t str_gag_range = 2;
double str_gag_zero = 0;

uint8_t shock_pot_range = 5;
double shock_pot_zero = 0;

uint16_t start_pixel = 360;
uint16_t end_pixel = 383;

void sendCanMessage(uint16_t id, uint8_t msg[]);