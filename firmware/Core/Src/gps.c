#include "gps.h"
#include <string.h>
#include <math.h>

extern UART_HandleTypeDef huart2; // L'UART de votre GPS (à vérifier selon votre main.c)

// --- Variables Globales ---
volatile int32_t lat_gps_actual = 0;
volatile int32_t lon_gps_actual = 0;
volatile uint8_t number_used_sats = 0;
volatile uint8_t fix_type = 0;
volatile uint8_t new_gps_data_available = 0;

// --- Variables Internes UBX ---
uint8_t gps_rx_byte;

// Variables pour l'interpolation
int32_t l_lat_gps = 0; // La position interpolée finale
int32_t l_lon_gps = 0;

static int32_t lat_gps_previous = 0;
static int32_t lon_gps_previous = 0;

static float lat_step = 0.0f;
static float lon_step = 0.0f;
static float lat_accumulated = 0.0f;
static float lon_accumulated = 0.0f;

static uint8_t interpolation_counter = 0;

typedef enum {
    SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
} ubx_state_t;

ubx_state_t ubx_state = SYNC1;
uint8_t msg_class, msg_id;
uint16_t msg_len, payload_idx;
uint8_t payload[100];
uint8_t calc_cka, calc_ckb, rec_cka;

#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t flags3;
    uint32_t reserved0;
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
} ubx_nav_pvt_t;
#pragma pack(pop)

ubx_nav_pvt_t drone_gps_data;

// --- Fonctions privées ---
static void calculate_checksum(uint8_t c) {
    calc_cka = calc_cka + c;
    calc_ckb = calc_ckb + calc_cka;
}

// --- Fonctions publiques ---
void GPS_Init(void) {
    // On lance l'écoute du premier octet en tâche de fond
    HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
}

void GPS_Process_Byte(uint8_t c) {
    switch (ubx_state) {
        case SYNC1:
            if (c == 0xB5) ubx_state = SYNC2;
            break;
        case SYNC2:
            if (c == 0x62) {
                ubx_state = CLASS;
                calc_cka = 0; calc_ckb = 0;
            } else ubx_state = SYNC1;
            break;
        case CLASS:
            msg_class = c;
            calculate_checksum(c);
            ubx_state = ID;
            break;
        case ID:
            msg_id = c;
            calculate_checksum(c);
            ubx_state = LEN1;
            break;
        case LEN1:
            msg_len = c;
            calculate_checksum(c);
            ubx_state = LEN2;
            break;
        case LEN2:
            msg_len |= (c << 8);
            calculate_checksum(c);
            if (msg_len > sizeof(payload)) {
                ubx_state = SYNC1;
            } else {
                payload_idx = 0;
                ubx_state = (msg_len > 0) ? PAYLOAD : CK_A;
            }
            break;
        case PAYLOAD:
            payload[payload_idx++] = c;
            calculate_checksum(c);
            if (payload_idx >= msg_len) ubx_state = CK_A;
            break;
        case CK_A:
            rec_cka = c;
            ubx_state = CK_B;
            break;
        case CK_B:
            if (calc_cka == rec_cka && calc_ckb == c) {
                // UBX-NAV-PVT
                if (msg_class == 0x01 && msg_id == 0x07 && msg_len == 92) {
                    memcpy(&drone_gps_data, payload, sizeof(ubx_nav_pvt_t));

                    // Mise à jour des variables globales pour le drone
                    lat_gps_actual = drone_gps_data.lat;
                    lon_gps_actual = drone_gps_data.lon;
                    number_used_sats = drone_gps_data.numSV;
                    fix_type = drone_gps_data.fixType;

                    // ON LÈVE LE DRAPEAU POUR LE SÉQUENCEUR
                    // (Le GPS est à 5Hz, donc ce drapeau se lèvera 5 fois par seconde)
                    new_gps_data_available = 1;
                }
            }
            ubx_state = SYNC1;
            break;
    }
}

void GPS_Interpolate(void) {

    // 1. QUAND UNE NOUVELLE DONNÉE ARRIVE DU SATELLITE (5 Fois par seconde)
    if (new_gps_data_available) {
        new_gps_data_available = 0; // On abaisse le drapeau

        // Au tout premier démarrage, on initialise les anciennes valeurs
        if (lat_gps_previous == 0 && lon_gps_previous == 0) {
            lat_gps_previous = lat_gps_actual;
            lon_gps_previous = lon_gps_actual;
            l_lat_gps = lat_gps_actual;
            l_lon_gps = lon_gps_actual;
        }

        // Calcul du "pas" d'interpolation.
        // Le GPS tourne à 5Hz. Notre boucle à 250Hz. Il y a donc 50 boucles entre chaque position GPS.
        // On divise la distance parcourue par 50 pour avoir un lissage parfait à chaque boucle (4ms).
        lat_step = (float)(lat_gps_actual - lat_gps_previous) / 50.0f;
        lon_step = (float)(lon_gps_actual - lon_gps_previous) / 50.0f;

        // On sauvegarde pour le prochain tour
        lat_gps_previous = lat_gps_actual;
        lon_gps_previous = lon_gps_actual;

        // On réinitialise l'accumulateur et le compteur
        lat_accumulated = 0.0f;
        lon_accumulated = 0.0f;
        interpolation_counter = 50;
    }

    // 2. À CHAQUE BOUCLE DE 4ms (250 Fois par seconde)
    if (interpolation_counter > 0) {
        interpolation_counter--;

        // On ajoute notre fraction de distance
        lat_accumulated += lat_step;
        lon_accumulated += lon_step;

        // Puisque l_lat_gps est un entier (int32_t), on ne lui donne que les valeurs entières de l'accumulateur
        if (fabsf(lat_accumulated) >= 1.0f) {
            l_lat_gps += (int32_t)lat_accumulated;
            lat_accumulated -= (int32_t)lat_accumulated; // On garde la virgule pour plus tard
        }

        if (fabsf(lon_accumulated) >= 1.0f) {
            l_lon_gps += (int32_t)lon_accumulated;
            lon_accumulated -= (int32_t)lon_accumulated;
        }
    }
}
