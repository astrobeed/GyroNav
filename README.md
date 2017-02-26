## Digital Horizon Compass
#####Rilevatore di assetto e orientamento per robot mobili.

Si tratta di un sensore assoluto che misura l'inclinazione sugli assi X e Y del piano e rileva l'orientamento rispetto al Nord magnetico. 

Il sistema è composto da una scheda Arduino Pro mini, fornisce la capacità di calcolo e la comunicazione, una IMU MPU6050 e un magnetometro HMC5883L, in alternativa un IMU 9 d.o.f. MPU9150.
L'assetto è ottenuto tramite il DMP del MPU6050, l'orientamento tramite il magnetometro, esterno oppure quello integrato nel MPU9150, il software esegue una sensor fusion tra le letture della IMU e quelle del magnetometro in modo da poter compensare il valore angolare del Nord magnetico in funzione dell'inclinazione del sensore.
