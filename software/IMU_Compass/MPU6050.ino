// estrae dati DMP
void MPU6050_Elaborate(void)
{
  /*
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    // Read the raw accel/gyro values from the MPU-6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    }
  */

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Obtain Euler angles from buffer
    // mpu.dmpGetQuaternion(&q, fifoBuffer);
    // mpu.dmpGetEuler(euler, &q);

    // Obtain YPR angles from buffer
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void MPU6050_init(void)
{
  // initialize MPU6050
#ifdef debug
  Serial.println(F("Initializing I2C devices..."));
#endif
  mpu.initialize();
#ifdef debug
  // verify connection
  Serial.println(F("Testing device connections..."));
  if (mpu.testConnection()) Serial.println(F("MPU6050 connection successful"));
  else Serial.println(F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
#ifdef debug
    Serial.println(F("Enabling DMP..."));
#endif
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    // enable MPU interrupt detection
#ifdef debug
    Serial.println(F("Enabling interrupt detection (AVR external interrupt 0)..."));
#endif
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef debug
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
    dmpReady = true;

    // Set the full scale range of the gyro
    uint8_t FS_SEL = 0;
    //mpu.setFullScaleGyroRange(FS_SEL);

    // get default full scale value of gyro - may have changed from default
    // function call returns values between 0 and 3
    uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
#ifdef debug
    Serial.print("FS_SEL = ");
    Serial.println(READ_FS_SEL);
#endif
    GYRO_FACTOR = 131.0 / (FS_SEL + 1);

    // get default full scale value of accelerometer - may not be default value.
    // Accelerometer scale factor doesn't reall matter as it divides out
    uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
#ifdef debug
    Serial.print("AFS_SEL = ");
    Serial.println(READ_AFS_SEL);
#endif
    //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);

    // Set the full scale range of the accelerometer
    // uint8_t AFS_SEL = 0;
    // mpu.setFullScaleAccelRange(AFS_SEL);

    // get expected DMP packet size for later comparison
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

