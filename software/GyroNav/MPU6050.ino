// estrae dati DMP
void MPU6050_Elaborate(void)
{
  // reset interrupt flag e INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // test overflow FiFo
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset FiFo
    mpu.resetFIFO();
    // errore FiFo, verificare tempi acquisizione dati DMP
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) {
    // attesa per pacchetto dati completo, tipicamente da 1 a 10 uS
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // lettura pacchetto dati dalla FiFo
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // scala conteggio FiFo
    fifoCount -= packetSize;

    // Lettura angoli di Eulero dal buffer
    // attualmente non utilizzato
    // mpu.dmpGetQuaternion(&q, fifoBuffer);
    // mpu.dmpGetEuler(euler, &q);

    // Lettura angoli YPR dal buffer
    // Y = Yaw, P = Pitch, R = Roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void MPU6050_init(void)
{
  // init MPU6050
#ifdef debug
  Serial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();
#ifdef debug
  // verifica connessione
  Serial.println(F("Testing device connections..."));
  if (mpu.testConnection()) Serial.println(F("MPU6050 connection successful"));
  else Serial.println(F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
#endif
  // carica firmware sul DMP
  devStatus = mpu.dmpInitialize();

  // verifica stato operativo DMP
  if (devStatus == 0) {
#ifdef debug
    Serial.println(F("Enabling DMP..."));
#endif
    // attiva il funzionamento del DMP
    mpu.setDMPEnabled(true);
    // attiva interrupt DMP, usa INT0
#ifdef debug
    Serial.println(F("Enabling interrupt detection (AVR external interrupt 0)..."));
#endif
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
#ifdef debug
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
    // setta il flag del DMP, permette l'esecuzione della loop() 
    dmpReady = true;

    // Setta la scal del giroscopio
    uint8_t FS_SEL = 0;
    uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
#ifdef debug
    Serial.print("FS_SEL = ");
    Serial.println(READ_FS_SEL);
#endif
    GYRO_FACTOR = 131.0 / (FS_SEL + 1);

    // Setta la scala dell'acceleromentro.
    uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
#ifdef debug
    Serial.print("AFS_SEL = ");
    Serial.println(READ_AFS_SEL);
#endif
    // ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
    // acquisisce la dimensione del paccheto dati DMP
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

