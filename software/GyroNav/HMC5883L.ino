HMC5883L mag;

byte MHC_init(void)
{
  byte stat = 0;
  // initialize HMC5883L

#ifdef debug
  Serial.println("Initializing HMC5883L...");
#endif
  mag.initialize();
  // verify connection
  stat = mag.testConnection();
#ifdef debug
  Serial.println(F("Testing device connections..."));
  if (stat) Serial.println(F("HMC5883L connected"));
  else Serial.println(F("HMC5883L connection failed"));
#endif
  return stat;
}

// Compass data, 66 ms cycle (15 Hz refresh)
int Compass_Data()
{
  // media mobile
  byte i;
  // direzione in radianti
  float heading;

  // read raw heading measurements from device
  mag.getHeading(&mx, &my, &mz);

#ifdef debug_mag
  // display tab-separated gyro x/y/z values
  Serial.print("mag:\t");
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.print(mz); Serial.print("\t");
#endif

  /*
    // heading, non compensato, in gradi, 0 gradi è il Nord.
    heading = atan2(my, mx);
    if (heading < 0)
      heading += 2 * M_PI;
  */

#ifdef debug_mag
  Serial.print("heading:\t");
  Serial.println(heading * RADIANS_TO_DEGREES);
#endif

  /*

    Compensa bussola in base a tilt e roll.
    ypr[1] = tilt IMU
    ypr[2] = roll IMU

    Formula usata per la compensazione
    Yh = XM * sin(Roll) * sin(Pitch) + YM * cos(Roll) - ZM * sin(Roll) * cos(Pitch)
    Xh = XM * cos(Pitch) + ZM * sin(Pitch)
    Heading = arctan(Yh/Xh)
  */

  //double Xh = mx * cos(ypr[1]) + my * sin(ypr[2]) * sin(ypr[1]) - mz * cos(ypr[2]) * sin(ypr[1]);
  //double Yh = my * cos(ypr[2]) - mz * sin(ypr[2]);

  // double Xh = mx * cos(-ypr[1]) + my * sin(ypr[2]) * sin(-ypr[1]) - mz * cos(ypr[2]) * sin(-ypr[1]);
  // double Yh = my * cos(ypr[2]) + mz * sin(ypr[2]);

  // axes adjust
  float Pitch = ypr[1];
  float Roll = ypr[2];
  float XM = my;
  float YM = mx;
  float ZM = mz;

  float Yh = XM * sin(Roll) * sin(Pitch) + YM * cos(Roll) - ZM * sin(Roll) * cos(Pitch);
  float Xh = XM * cos(Pitch) + ZM * sin(Pitch);

  heading = atan2( Yh, Xh );
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;

  // conversione tra radianti e gradi
  heading_deg = heading * RADIANS_TO_DEGREES;
  // correzione per la declinazione magnetica
  heading_deg -= MAG_DECLI;

  // acquisizione valori media mobile
  mag_media[idx_mag] = heading_deg;
  idx_mag++;
  if (idx_mag > 16) idx_mag = 0;
  // calcolo media mobile
  heading_deg = 0;
  for (i = 0; i < 16; i++) heading_deg += mag_media[i];
  // disione per 16 e verso di rotazione
#ifdef MAG_REVERSE
  heading_deg = 360 - (heading_deg >> 4);
#else
  heading_deg = (heading_deg >> 4);
#endif
}

