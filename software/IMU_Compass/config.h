/*
    modalità out dati
    0 -> Gui processing, valori in gradi (default)
    1 -> Gui processing, valori in radianti
    2 -> formato ascii csv
    3 -> formato binario
*/

#define datmode_0
// #define datmode_1
// #define datmode_2
// #define datmode_3

/*
  Tipo di IMU usata
  6050 = MPU6050 + HMC5883L (default)
  9150 = MPU9150 (sperimentale, non ancora supportato)
*/
#define IMU_6050
// #define IMU_9150

// attiva informazioni debug su serial monitor
// commentare per l'uso normale
#define debug
// #define debug_mag
