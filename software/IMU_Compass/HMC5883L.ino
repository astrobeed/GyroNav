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
  // compass variable.
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
    // heading, non compensato, in gradie, 0 degree è il Nord.
    heading = atan2(my, mx);
    if (heading < 0)
      heading += 2 * M_PI;
  */

#ifdef debug_mag
  Serial.print("heading:\t");
  Serial.println(heading * 180 / M_PI);
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

  // int headingDegrees = heading * 180 / M_PI;
  // headingDegrees = heading * 180 / M_PI;
  heading_deg = heading * 180 / M_PI;
}

