// V3 experimental virtual WAS, diagnostic mode only. No KEYA actuation in virtual mode.
// HWT TTL TX -> Teensy RX5 pin 21. GPS Serial2, TM171 Serial7.
static HardwareSerial &HWT_SERIAL = Serial5;
static const bool HWT_BENCH_LOCK = true;
static const uint32_t HWT_BAUD = 9600, HWT_TIMEOUT_MS = 350;
static const float HWT_SIGN = 1.0f, TM171_SIGN = 1.0f;
static uint8_t hwtBuf[11], hwtIdx = 0;
static bool hwtSeen = false, hwtZeroValid = false, hwtPairInit = false, hwtGpsInit = false;
static uint32_t hwtMs = 0, hwtSeq = 0, hwtUsed = 0, pairMs = 0, gpsMs = 0, stableMs = 0;
static float hwtYaw = 0, lastHwt = 0, lastTm = 0, hwtRelative = 0, hwtZero = 0, hwtWas = 0;
static float hwtStraight = 0, gpsBase = 0, gpsRate = 999, tmRate = 999, relRate = 999;
static const char *hwtState = "WAIT_HWT";
static uint32_t jumpMs = 0;
static bool jumpSeen = false;

static float HWTwrap(float a) {
  if (a > 180.0f) a -= 360.0f;
  if (a < -180.0f) a += 360.0f;
  return a;
}
void HWTinvalidate() {
  hwtZeroValid = false; hwtPairInit = false; hwtGpsInit = false;
  hwtStraight = 0; stableMs = 0; hwtWas = 0; hwtRelative = 0;
  gpsRate = 999; tmRate = 999; relRate = 999;
  hwtUsed = hwtSeq; wasZeroDone = false;
}
static bool HWTalive() { return !nmeaoutput && hwtSeen && millis()-hwtMs < HWT_TIMEOUT_MS; }
void HWTsetup() {
  if (!nmeaoutput) HWT_SERIAL.begin(HWT_BAUD);
  Serial.println("[HWT V3] Serial5 9600; diagnostic motor lock");
}
void HWTprocess() {
  if (nmeaoutput) return;
  while (HWT_SERIAL.available()) {
    uint8_t b = HWT_SERIAL.read();
    if (!hwtIdx && b != 0x55) continue;
    hwtBuf[hwtIdx++] = b;
    if (hwtIdx == 11) {
      uint8_t sum = 0; for (uint8_t i=0;i<10;i++) sum += hwtBuf[i];
      if (sum == hwtBuf[10]) {
        if (hwtBuf[1] == 0x53) {
          uint32_t now = millis();
          if (hwtSeen && now-hwtMs >= HWT_TIMEOUT_MS) HWTinvalidate();
          int16_t raw = (int16_t)((uint16_t)hwtBuf[7]<<8 | hwtBuf[6]);
          hwtYaw = raw * (180.0f/32768.0f);
          hwtMs = now; hwtSeen = true; ++hwtSeq;
        }
        hwtIdx = 0;
      } else {
        // Resynchronise after a damaged frame.
        uint8_t start = 1;
        while (start < 11 && hwtBuf[start] != 0x55) ++start;
        hwtIdx = 11-start;
        memmove(hwtBuf,hwtBuf+start,hwtIdx);
      }
    }
  }
}
bool HWTwasReady() { return steerConfig.IsDanfoss && HWTalive() && TM171yawFresh() && hwtZeroValid; }
bool HWTwasCanSteer() { return !HWT_BENCH_LOCK && HWTwasReady(); }
float HWTwasGetAngle() { return hwtWas; }
float HWTzeroDistance() { return hwtStraight; }
const char* HWTwasStatus() {
  if (!steerConfig.IsDanfoss) return "ANALOG";
  if (nmeaoutput) return "PORT_CONFLICT";
  if (!HWTalive()) return "NO_HWT";
  if (!TM171yawFresh()) return "NO_TM171";
  if (jumpSeen && millis()-jumpMs < 2000) return "ANGLE_FAULT";
  if (HWTwasReady()) return "READY";
  if (!HWTgpsFresh()) return "WAIT_GPS";
  return hwtState;
}
void HWTwasUpdate(float tmYawDeg, float speedKmh, bool manual) {
  uint32_t now = millis();
  if (!HWTalive() || !TM171yawFresh() || !isfinite(tmYawDeg)) {
    HWTinvalidate(); hwtState = "WAIT_SENSORS"; return;
  }
  if (!HWTgpsFresh()) { hwtGpsInit=false; gpsRate=999; hwtStraight=0; stableMs=0; }
  else if (!hwtGpsInit) {
    gpsBase=HWTgpsHeading(); gpsMs=HWTgpsStamp(); hwtGpsInit=true; gpsRate=999;
  } else if (HWTgpsStamp()-gpsMs >= 1000) {
    uint32_t elapsed = HWTgpsStamp()-gpsMs;
    gpsRate = fabsf(HWTwrap(HWTgpsHeading()-gpsBase))*1000.0f/elapsed;
    gpsBase=HWTgpsHeading(); gpsMs=HWTgpsStamp();
  }
  if (hwtSeq == hwtUsed) return;
  hwtUsed = hwtSeq;
  // The received timestamps are approximate; require a recent matching body sample.
  if (TM171yawAge() > 120) { HWTinvalidate(); hwtState="WAIT_SENSORS"; return; }
  if (!hwtPairInit) {
    lastHwt=hwtYaw; lastTm=tmYawDeg; pairMs=hwtMs; hwtPairInit=true; return;
  }
  uint32_t ms=hwtMs-pairMs;
  if (!ms) return;
  float dt=ms*0.001f;
  float dh=HWTwrap(hwtYaw-lastHwt)*HWT_SIGN;
  float dtm=HWTwrap(tmYawDeg-lastTm)*TM171_SIGN;
  float dr=dh-dtm;
  if (ms>=HWT_TIMEOUT_MS || fabsf(dh)>25 || fabsf(dtm)>25 || fabsf(dr)>15) {
    HWTinvalidate(); jumpSeen=true; jumpMs=now; hwtState="ANGLE_FAULT"; return;
  }
  lastHwt=hwtYaw; lastTm=tmYawDeg; pairMs=hwtMs;
  hwtRelative=HWTwrap(hwtRelative+dr);
  tmRate=tmRate==999?dtm/dt:0.8f*tmRate+0.2f*dtm/dt;
  relRate=relRate==999?dr/dt:0.8f*relRate+0.2f*dr/dt;
  if (hwtZeroValid) {
    hwtWas=HWTwrap(hwtRelative-hwtZero);
    if (fabsf(hwtWas)>55) { HWTinvalidate(); jumpSeen=true; jumpMs=now; return; }
  }
  bool straight=manual && HWTgpsFresh() && speedKmh>=4 && HWTgpsSpeed()>=4 &&
    fabsf(speedKmh-HWTgpsSpeed())<=2 && gpsRate<=0.7f && fabsf(tmRate)<=0.3f &&
    fabsf(relRate)<=0.4f && fabsf(dtm)<=0.5f && fabsf(dr)<=0.5f;
  if (!straight) {
    hwtStraight=0; stableMs=0;
    hwtState=manual?"WAIT_STRAIGHT":"TURN_AOG_OFF";
  } else {
    if (!stableMs) stableMs=now;
    hwtStraight += HWTgpsSpeed()/3.6f*dt;
    hwtState="ZEROING";
    if (hwtStraight>=8 && now-stableMs>=3000) {
      if (!hwtZeroValid) {
        hwtZero=hwtRelative; hwtWas=0; hwtZeroValid=true;
        Serial.println("[HWT V3] ZERO OK (estimated from straight travel)");
      }
      // No moving zero / drift correction in this diagnostic release.
      hwtStraight=8;
    }
  }
}
void HWTsendStatus() {
  static uint32_t last=0;
  if (!Ethernet_running || millis()-last<250) return;
  last=millis();
  char msg[180];
  const char *state=HWTwasStatus();
  int n=snprintf(msg,sizeof(msg),"WAS3|%s|%.2f|%.1f|%lu|%lu|%s",state,
    (double)steerAngleActual,(double)hwtStraight,
    (unsigned long)(hwtSeen?millis()-hwtMs:99999UL),(unsigned long)TM171yawAge(),
    steerConfig.IsDanfoss?"BENCH":"ANALOG");
  if(n>0 && n<(int)sizeof(msg)) {
    Eth_udpPAOGI.beginPacket(Eth_ipDestination,17999);
    Eth_udpPAOGI.write((const uint8_t*)msg,n);
    Eth_udpPAOGI.endPacket();
  }
}
