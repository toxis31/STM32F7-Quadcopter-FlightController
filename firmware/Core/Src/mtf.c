/* USER CODE BEGIN Header */
#include "mtf.h"
#include <string.h>

// On importe le handle de l'UART5 configuré dans le main.c
extern UART_HandleTypeDef huart5;

// --- DÉFINITIONS MICOLINK INTERNES ---
#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             MICOLINK_MAX_PAYLOAD_LEN + 7

enum {
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,
};

typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

#pragma pack(push, 1)
typedef struct {
    uint32_t  time_ms;
    uint32_t  distance;
    uint8_t   strength;
    uint8_t   precision;
    uint8_t   dis_status;
    uint8_t   reserved1;
    int16_t   flow_vel_x;
    int16_t   flow_vel_y;
    uint8_t   flow_quality;
    uint8_t   flow_status;
    uint16_t  reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack(pop)
// ------------------------------------

// --- VARIABLES GLOBALES ---
uint8_t mtf_rx_byte; // Octet lu par l'interruption

uint32_t sensor_distance_mm = 0;
uint32_t sensor_distance_filtered_mm = 0.0f;
int16_t  sensor_flow_x = 0;
int16_t  sensor_flow_y = 0;
uint8_t  sensor_flow_quality = 0;
uint8_t  sensor_distance_status = 0;

// --- PROTOTYPES PRIVÉS ---
static bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data);
static bool micolink_check_sum(MICOLINK_MSG_t* msg);

// --- FONCTIONS PUBLIQUES ---

void MTF_Init(void) {
    // Séquence d'initialisation pour forcer le module en protocole Micolink
	uint8_t micolink_init_cmd[] = {0xEF, 0x01, 0x00, 0x02, 0x24, 0x12, 0x02, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x04, 0x09, 0x00, 0x00, 0x01, 0x58, 0x00, 0x00, 0x00, 0x00, 0xF4};

    // Envoi de la trame de configuration sur l'UART5
    HAL_UART_Transmit(&huart5, micolink_init_cmd, sizeof(micolink_init_cmd), HAL_MAX_DELAY);

    // Petite pause pour laisser le capteur digérer la configuration
    HAL_Delay(50);

    // Lancement de la réception UART en mode Interruption (1 octet à la fois)
    HAL_UART_Receive_IT(&huart5, &mtf_rx_byte, 1);
}

void MTF_Process_Byte(uint8_t data) {
    static MICOLINK_MSG_t msg;

    if(micolink_parse_char(&msg, data) == false)
        return;

    switch(msg.msg_id) {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
            memcpy(&payload, msg.payload, msg.len);

            sensor_distance_mm     = payload.distance; /////////////////// + 0
            sensor_distance_status = payload.dis_status;

            // --- LE FILTRE PASSE-BAS ---
			if (sensor_distance_filtered_mm == 0.0f) {
				// Initialisation au premier démarrage pour éviter que le filtre ne parte de 0
				sensor_distance_filtered_mm = (float)payload.distance;
				sensor_distance_mm = sensor_distance_filtered_mm + 54;
			} else {
				// 80% de l'ancienne valeur (lissage) + 20% de la nouvelle valeur (réactivité)
				sensor_distance_filtered_mm = (sensor_distance_filtered_mm * 0.7f) + ((float)payload.distance * 0.3f);
				sensor_distance_mm = sensor_distance_filtered_mm + 54;
			}

            sensor_flow_x          = payload.flow_vel_x;
            sensor_flow_y          = payload.flow_vel_y;
            sensor_flow_quality    = payload.flow_quality;
            break;
        }
        default:
            break;
    }
}

// --- FONCTIONS PRIVÉES ---

static bool micolink_check_sum(MICOLINK_MSG_t* msg) {
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i=0; i<length; i++) {
        checksum += temp[i];
    }

    return (checksum == msg->checksum);
}

static bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data) {
    switch(msg->status) {
    case 0:
        if(data == MICOLINK_MSG_HEAD) {
            msg->head = data;
            msg->status++;
        }
        break;
    case 1: msg->dev_id = data; msg->status++; break;
    case 2: msg->sys_id = data; msg->status++; break;
    case 3: msg->msg_id = data; msg->status++; break;
    case 4: msg->seq = data; msg->status++; break;
    case 5:
        msg->len = data;
        if(msg->len == 0) msg->status += 2;
        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN) msg->status = 0;
        else msg->status++;
        break;
    case 6:
        msg->payload[msg->payload_cnt++] = data;
        if(msg->payload_cnt == msg->len) {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;
    case 7:
        msg->checksum = data;
        msg->status = 0;
        if(micolink_check_sum(msg)) return true;
        break;
    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }
    return false;
}
/* USER CODE END Header */
