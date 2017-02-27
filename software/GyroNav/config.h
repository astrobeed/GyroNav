/*
    modalità out dati
    0 -> Gui processing, valori in gradi (default)
    1 -> Gui processing, valori in radianti
    2 -> formato ascii per serial monitor
    3 -> formato binario (todo)
*/

// costante conversione da radianti a gradi
const float RADIANS_TO_DEGREES = 57.2958; // 180/3.14159

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

// Declinazione magnetica
#define MAG_DECLI 0.0
// inverte il verso di rotazione della bussola se necessario
#define MAG_REVERSE


