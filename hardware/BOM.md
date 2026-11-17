# Nomenclature de la carte contrôleur de vol (schéma `FlightController.SchDoc`)

Carte porteuse conçue sous Altium Designer : le STM32F722RET6 est soudé sur la carte,
les capteurs et la radio sont des modules externes raccordés par connecteurs.

| Repère | Élément | Rôle |
|---|---|---|
| IC1 | STM32F722RET6 (Cortex-M7, FPU simple précision) | Microcontrôleur du contrôleur de vol |
| J10 | Connecteur d'alimentation 2 pôles | Arrivée batterie LiPo 3S (11,1 V) |
| J4, J16 | Connecteurs 2 broches | Régulateur à découpage 3,3 V (module) |
| J5 | Connecteur 8 broches | Module IMU (gyroscope / accéléromètre), bus I²C + interruption |
| J12, J13 | Connecteurs 4 + 3 broches | Baromètre LPS22HH (I²C dans la version finale) + interruption |
| J3 | Connecteur 6 broches | Module GPS + boussole (UART + I²C) |
| J2 | Connecteur 4 broches | Lidar / capteur de flux optique (UART) |
| J1 | Connecteur 4 broches | Récepteur radio ExpressLRS (UART) |
| J7, J8, J9, J11 | Connecteurs 3 broches | Sorties PWM vers les 4 ESC |
| J6 | Connecteur 6 broches | Câble USB-UART (télémétrie / debug) |
| J14, J15 | Connecteurs 3 broches | Programmation / debug SWD (sonde Segger J-Link) |
| JMP1 | Cavalier 3 broches | Choix de la source d'alimentation : BEC 5 V des ESC ou batterie |
| LD1 – LD4 | LED | Signalisation d'état |
| — | Bouton poussoir | Entrée utilisateur |
| TP1 – TP9 | Points de test | Mesures pendant la mise au point |
| C1 – C6 | Condensateurs 100 nF | Découplage du STM32 |
| R1, R2 | 3,9 kΩ / 1,2 kΩ | Pont diviseur de mesure de la tension batterie |
| R3 – R6 | 2,2 kΩ | Résistances des LED |
| R7 | — | Voir schéma |

Signaux nommés sur le schéma : `SDA/SCL_gyro`, `INT_gyro`, `SDA/CLK_Baro`, `INT_Baro`,
`SDA/SCL_Boussole`, `Tx/Rx_GPS`, `Tx/Rx_Lidar`, `Tx/Rx_Manette`, `Tx/Rx_UART`,
`ESC1…ESC4`, `SWDIO/SWCLK`, `LED_1…3`, `Bouton`, `valeur batterie`.
