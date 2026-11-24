/*
  HWT101CT_WAS.ino
  Virtual WAS: HWT101CT na zwrotnicy + TM171 na ciagniku.

  Danfoss OFF -> oryginalny ADS1115 WAS.
  Danfoss ON  -> ten virtual WAS.

  HWT101CT TTL:
    TX HWT -> Teensy 4.1 RX5 pin 21
    RX HWT -> Teensy 4.1 TX5 pin 20 (opcjonalnie)
    GND wspolne

  Serial5 jest wolny tylko gdy nmeaoutput == 0.

  Fabryczny HWT101CT TTL:
    9600 baud, 10 Hz.
  Dlatego ta pierwsza wersja jest przygotowana pod ustawienia fabryczne.
  Po testach przejdziemy na 100 Hz.
*/

static HardwareSerial &HWT_SERIAL = Serial5;

static const uint32_t HWT_BAUD = 9600;
static const uint32_t HWT_TIMEOUT_MS = 500;

static const float HWT_SIGN = +1.0f;
static const float TM171_SIGN = +1.0f;

static const float HWT_WAS_LIMIT_DEG = 55.0f;

static const float HWT_ZERO_MIN_SPEED_KMH = 4.0f;
static const float HWT_ZERO_DISTANCE_M = 8.0f;
static const float HWT_ZERO_MAX_TM_RATE_DPS = 0.30f;
static const float HWT_ZERO_MAX_GPS_RATE_DPS = 0.30f;
static const float HWT_ZERO_MAX_REL_RATE_DPS = 0.40f;
static const float HWT_ZERO_DRIFT_CORRECTION = 0.15f;

static uint8_t hwtBuf[11];
static uint8_t hwtIdx = 0;

static float hwtAngleZDeg = 0.0f;
static bool hwtSeen = false;
static uint32_t hwtLastFrameMs = 0;
static uint32_t hwtFrameCounter = 0;
static uint32_t hwtFrameCounterUsed = 0;

static bool hwtEstimatorInit = false;
static float hwtLastAngleZDeg = 0.0f;
static float hwtLastTmYawDeg = 0.0f;
static uint32_t hwtLastEstimatorMs = 0;

static float hwtWasDeg = 0.0f;
static float hwtRelRateDps = 0.0f;
static float hwtTmRateDps = 0.0f;
static float hwtGpsRateDps = 999.0f;

static bool hwtGpsInit = false;
static float hwtLastGpsHeadingDeg = 0.0f;
static uint32_t hwtLastGpsMs = 0;

static bool hwtZeroValid = false;
static float hwtStraightDistanceM = 0.0f;
static uint32_t hwtStraightLastMs = 0;

static uint32_t hwtLastDebugMs = 0;

static float HWT_wrap180(float a)
{
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static bool HWT_checksumOK(const uint8_t *p)
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 10; i++) sum += p[i];
  return sum == p[10];
}

static int16_t HWT_s16(uint8_t lo, uint8_t hi)
{
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

void HWTsetup()
{
  if (nmeaoutput)
  {
    Serial.println("[HWT] BLAD: Serial5 zajety przez NMEA OUT");
    return;
  }

  HWT_SERIAL.begin(HWT_BAUD);
  delay(20);

  Serial.print("[HWT] Serial5 start baud=");
  Serial.println(HWT_BAUD);
}

void HWTprocess()
{
  if (nmeaoutput) return;
  while (HWT_SERIAL.available())
  {
    uint8_t b = HWT_SERIAL.read();

    if (hwtIdx == 0)
    {
      if (b == 0x55)
      {
        hwtBuf[0] = b;
        hwtIdx = 1;
      }
      continue;
    }

    hwtBuf[hwtIdx++] = b;

    if (hwtIdx >= 11)
    {
      hwtIdx = 0;

      if (!HWT_checksumOK(hwtBuf)) continue;

      // Standard WIT angle frame 0x53:
      // Z/Yaw = bytes 6,7, scale raw/32768*180 deg.
      if (hwtBuf[1] == 0x53)
      {
        int16_t rawZ = HWT_s16(hwtBuf[6], hwtBuf[7]);
        hwtAngleZDeg = ((float)rawZ / 32768.0f) * 180.0f;
        hwtLastFrameMs = millis();
        hwtSeen = true;
        hwtFrameCounter++;
      }
    }
  }
}

void HWTwasUpdate(float tmYawDeg, float speedKmh, float gpsHeadingDeg)
{
  uint32_t now = millis();

  if (!useTM171)
  {
    hwtZeroValid = false;
    hwtStraightDistanceM = 0.0f;
    return;
  }

  // Wolne, niezalezne potwierdzenie jazdy prosto z headingu GNSS.
  if (!hwtGpsInit)
  {
    hwtLastGpsHeadingDeg = gpsHeadingDeg;
    hwtLastGpsMs = now;
    hwtGpsInit = true;
  }
  else if ((now - hwtLastGpsMs) >= 200)
  {
    float dtGps = (now - hwtLastGpsMs) * 0.001f;
    float dGps = HWT_wrap180(gpsHeadingDeg - hwtLastGpsHeadingDeg);
    hwtGpsRateDps = fabsf(dGps / dtGps);
    hwtLastGpsHeadingDeg = gpsHeadingDeg;
    hwtLastGpsMs = now;
  }

  // Każda nowa ramka HWT aktualizuje wzgledny kat zwrotnicy.
  if (hwtFrameCounter != hwtFrameCounterUsed)
  {
    hwtFrameCounterUsed = hwtFrameCounter;

    if (!hwtEstimatorInit)
    {
      hwtLastAngleZDeg = hwtAngleZDeg;
      hwtLastTmYawDeg = tmYawDeg;
      hwtLastEstimatorMs = now;
      hwtEstimatorInit = true;
    }
    else
    {
      float dt = (now - hwtLastEstimatorMs) * 0.001f;
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.5f) dt = 0.5f;

      float dHwt = HWT_wrap180(hwtAngleZDeg - hwtLastAngleZDeg) * HWT_SIGN;
      float dTm = HWT_wrap180(tmYawDeg - hwtLastTmYawDeg) * TM171_SIGN;
      float dRel = dHwt - dTm;

      if (fabsf(dRel) < 15.0f)
      {
        hwtWasDeg += dRel;
        hwtRelRateDps = dRel / dt;
      }

      hwtTmRateDps = dTm / dt;

      hwtLastAngleZDeg = hwtAngleZDeg;
      hwtLastTmYawDeg = tmYawDeg;
      hwtLastEstimatorMs = now;

      hwtWasDeg = constrain(hwtWasDeg, -HWT_WAS_LIMIT_DEG, HWT_WAS_LIMIT_DEG);
    }
  }

  bool alive = hwtSeen && ((now - hwtLastFrameMs) < HWT_TIMEOUT_MS);

  if (!alive)
  {
    hwtStraightDistanceM = 0.0f;
    hwtStraightLastMs = now;
    return;
  }

  if (hwtStraightLastMs == 0) hwtStraightLastMs = now;

  float dtStraight = (now - hwtStraightLastMs) * 0.001f;
  hwtStraightLastMs = now;
  if (dtStraight > 0.2f) dtStraight = 0.2f;

  bool straight =
      speedKmh >= HWT_ZERO_MIN_SPEED_KMH &&
      fabsf(hwtTmRateDps) <= HWT_ZERO_MAX_TM_RATE_DPS &&
      hwtGpsRateDps <= HWT_ZERO_MAX_GPS_RATE_DPS &&
      fabsf(hwtRelRateDps) <= HWT_ZERO_MAX_REL_RATE_DPS;

  if (straight)
  {
    hwtStraightDistanceM += (speedKmh / 3.6f) * dtStraight;

    if (hwtStraightDistanceM >= HWT_ZERO_DISTANCE_M)
    {
      if (!hwtZeroValid)
      {
        hwtWasDeg = 0.0f;
        hwtZeroValid = true;
        Serial.println("[HWT-WAS] ZERO OK po jezdzie prosto");
      }
      else
      {
        hwtWasDeg *= (1.0f - HWT_ZERO_DRIFT_CORRECTION);
      }

      hwtStraightDistanceM = 0.0f;
    }
  }
  else
  {
    hwtStraightDistanceM = 0.0f;
  }

  if ((now - hwtLastDebugMs) >= 2000)
  {
    hwtLastDebugMs = now;
    Serial.print("[HWT-WAS] HWT=");
    Serial.print(hwtAngleZDeg, 2);
    Serial.print(" TM=");
    Serial.print(tmYawDeg, 2);
    Serial.print(" tmRate=");
    Serial.print(hwtTmRateDps, 2);
    Serial.print(" gpsRate=");
    Serial.print(hwtGpsRateDps, 2);
    Serial.print(" relRate=");
    Serial.print(hwtRelRateDps, 2);
    Serial.print(" WAS=");
    Serial.print(hwtWasDeg, 2);
    Serial.print(" zero=");
    Serial.print(hwtZeroValid ? "OK" : "WAIT");
    Serial.print(" straight=");
    Serial.print(hwtStraightDistanceM, 1);
    Serial.println("m");
  }
}

bool HWTwasReady()
{
  return useTM171 &&
         hwtZeroValid &&
         hwtSeen &&
         ((millis() - hwtLastFrameMs) < HWT_TIMEOUT_MS);
}

float HWTwasGetAngle()
{
  return hwtWasDeg;
}
