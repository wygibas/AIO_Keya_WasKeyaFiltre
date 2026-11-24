/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/



////////////////// User Settings /////////////////////////
float fRoll = 0, fPitch = 0;
bool rpInitialized = false;

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

// -----------------------------------------------------------------------
// SELECTION DES SOURCES DE CAP POUR L'AUTO-ZERO WAS
// Valeurs par defaut - modifiables via interface web ou menu serie
// -----------------------------------------------------------------------

/*  PWM Frequency ->
	 490hz (default) = 0
	 122hz = 1
	 3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2484


//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34

//Define sensor pin for current or pressure sensor
//#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A10

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

#include <IPAddress.h>
#include "BNO08x_AOG.h"


#ifdef ARDUINO_TEENSY41
// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#ifdef ARDUINO_TEENSY41
//uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data
#endif

// Structure parametres auto-zero - definie ici, instance dans zAutoZeroMenu.ino
struct AutoZeroParams {
  float    speedMin;
  float    yawRateMax;
  float    gpsHdgMax;
  uint32_t timeSlowMs;
  uint32_t timeFastMs;
  float    speedSlow;
  float    speedFast;
  uint8_t  useBno;      // 1 = utiliser yaw rate BNO comme condition de stabilite
  uint8_t  useGps;      // 1 = utiliser variation cap GPS comme condition de stabilite
  float    beta;        // vitesse correction douce guidage actif (0.01=lent .. 0.2=rapide)
  uint16_t ident;
};
extern AutoZeroParams azParams;



//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
uint8_t watchdogKeya = 0;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };

// Hello from Machine - PGN 123 pour la détection par AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;
bool wasAogSeen = false;
bool wasAdcAvailable = false;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;
 bool isPGNFound = false, isHeaderFound = false;
 uint8_t pgn = 0, dataLength = 0, idx = 0;
boolean goDown = false, endDown = false , bitState = false, bitStateOld = false;  //CAN Hitch Control
byte hydLift = 0;
byte goPress[8]        = {0x15, 0x33, 0x1E, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press big go
byte goLift[8]         = {0x15, 0x33, 0x1E, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift big go
byte endPress[8]       = {0x15, 0x34, 0x1E, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press big end
byte endLift[8]        = {0x15, 0x34, 0x1E, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift big end

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//CAN Bus
bool engageCAN = false;          //Variable for Engage from CAN
bool workCAN = false;
long unsigned int lastIdActive = 0;
uint8_t KBUSRearHitch = 250;    //Variable for hitch height from KBUS (0-250 *0.4 = 0-100%) - CaseIH tractor bus

uint8_t countHyd = 0; // Compteur pour le temps d'appui bouton Fendt

uint32_t myTime;
uint32_t lastpush;
uint32_t Time;
uint32_t relayTime;

uint8_t lastHydLift = 0;
uint32_t lastpushbutton = 0;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
	uint8_t Kp = 40;              // proportional gain
	uint8_t lowPWM = 10;          // band of no action
	int16_t wasOffset = 0;
	uint8_t minPWM = 9;
	uint8_t highPWM = 60;         // max PWM value
	float steerSensorCounts = 30;
	float AckermanFix = 1;        // sent as percent
};  Storage steerSettings;      // 11 bytes (AW: 14 surely?


//Variables for config - 0 is false  
struct Config {
    uint8_t raiseTime = 2;
    uint8_t lowerTime = 4;
    uint8_t enableToolLift = 0;
    uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

    uint8_t user1 = 0; //user defined values set in machine tab
    uint8_t user2 = 0;
    uint8_t user3 = 0;
    uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes

void steerSettingsInit()
{
	// for PWM High to Low interpolator
	highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}


void autosteerSetup()
{
	//keep pulled high and drag low to activate, noise free safe
	pinMode(WORKSW_PIN, INPUT_PULLUP);
	pinMode(STEERSW_PIN, INPUT_PULLUP);

	// Disable digital inputs for analog input pins
	//pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
	pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

	//set up communication
	Wire1.end();
	Wire1.begin();

	//50Khz I2C
	//TWBR = 144;   //Is this needed?

	EEPROM.get(0, EEread);              // read identifier

	if (EEread != EEP_Ident)            // check on first start and write EEPROM
	{
		EEPROM.put(0, EEP_Ident);
		EEPROM.put(10, steerSettings);
		EEPROM.put(40, steerConfig);
		EEPROM.put(60, networkAddress);
    EEPROM.get(70, aogConfig);
	}
	else
	{
		EEPROM.get(10, steerSettings);     // read the Settings
		EEPROM.get(40, steerConfig);
		EEPROM.get(60, networkAddress);
    EEPROM.get(70, aogConfig);
	}

  wasAdcAvailable = adc.testConnection();
  Serial.println(wasAdcAvailable ? "ADC Connection OK" : "ADC absent: analog WAS locked");
	steerSettingsInit();
  if (sectionout == 1)
      {
        sectionControlSetup(); // ajoutés par FlorianT
      }

  // Restaurer l'offset WAS sauvegarde entre les redemarrages
  {
    float savedOffset = 0.0f;
    EEPROM.get(EEPROM_ADDR_WAS_OFFSET_F, savedOffset);
    if (!isnan(savedOffset) && !isinf(savedOffset) && fabsf(savedOffset) < 30.0f)
      wasOffsetF = savedOffset;
    else
      wasOffsetF = 0.0f;
    steerSettings.wasOffset = (int16_t)round(wasOffsetF * steerSettings.steerSensorCounts);
    Serial.print("WAS auto-zero offset restaure : ");
    Serial.println(wasOffsetF, 3);
  }

  // Restaurer le ratio ticks/degre Keya (calibration mecanique)
  {
    float savedTicks = 0.0f;
    EEPROM.get(EEPROM_ADDR_KEYA_TICKS, savedTicks);
    if (!isnan(savedTicks) && !isinf(savedTicks) && savedTicks > 1.0f && savedTicks < 500.0f)
      keyaTicksPerDeg = savedTicks;
    else
      keyaTicksPerDeg = KEYA_TICKS_PER_DEG_DEFAULT;
    Serial.print("Keya ticks/deg : ");
    Serial.println(keyaTicksPerDeg, 1);
  }
  wasZeroDone = false; // le zero doit etre etabli a chaque demarrage
  
	if (Autosteer_running)
	{
		Serial.println("Autosteer running, waiting for AgOpenGPS");
	}
	else
	{
		Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
		//    if(!Ethernet_running)Serial.println("Ethernet not available");
		Serial.println("Autosteer disabled, GPS only mode");
		return;
	}
  if (wasAdcAvailable) {
	adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS);
	adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
  }

  // Charger les parametres auto-zero depuis EEPROM
  azMenuSetup();
  emaParamsLoad();

}// End of Setup

float azCorrAccum = 0.0f; // accumulation sub-tick pour recalage progressif

bool HWTwebAllowed() {
  return watchdogTimer >= WATCHDOG_THRESHOLD || steerConfig.IsDanfoss;
}

void autosteerLoop()
{
#ifdef ARDUINO_TEENSY41
	ReceiveUdp();
#endif

  // Menu serie parametres auto-zero (touche 'm' dans le moniteur serie)
  // Legacy blocking serial menu is not allowed during steering or virtual WAS.
  if (!steerConfig.IsDanfoss && watchdogTimer >= WATCHDOG_THRESHOLD && azMenuLoop()) { pwmDrive = 0; motorDrive(); return; }

	//Serial.println("AutoSteer loop");

	// Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
	currentTime = systick_millis_count;

	if (currentTime - autsteerLastTime >= LOOP_TIME)
	{
		autsteerLastTime = currentTime;

		//reset debounce
		encEnable = true;

		//If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
		if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    if (watchdogKeya++ > 200) {
      watchdogKeya = 0;
      //digitalWrite(PWM2_RPWM, LOW);
    }

		//read all the switches
		workSwitch = digitalRead(WORKSW_PIN);  // read work switch
    if (workCAN == 1) workSwitch = 0;         // If CAN workswitch is on, set workSwitch ON

		if (steerConfig.SteerSwitch == 1)         //steer switch on - off
		{
			steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
		}
		else if (steerConfig.SteerButton == 1)    //steer Button momentary
		{
			reading = digitalRead(STEERSW_PIN);
      if (engageCAN)
      {
        reading = LOW;              //CAN Engage is ON (Button is Pressed)
        engageCAN = false;          //mod test thibault
      }
			if (reading == LOW && previous == HIGH)
			{
				if (currentState == 1)
				{
					currentState = 0;
					steerSwitch = 0;
				}
				else
				{
					currentState = 1;
					steerSwitch = 1;
				}
			}
			previous = reading;
		}
		else                                      // No steer switch and no steer button
		{
			// So set the correct value. When guidanceStatus = 1,
			// it should be on because the button is pressed in the GUI
			// But the guidancestatus should have set it off first
			if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
			{
				steerSwitch = 0;
				previous = 1;
			}

			// This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
			if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
			{
				steerSwitch = 1;
				previous = 0;
			}
		}

		if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
		{
			steerSwitch = 1; // reset values like it turned off
			currentState = 1;
			previous = 0;
		}

		// Pressure sensor?
		if (steerConfig.PressureSensor)
		{
			sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
			sensorSample *= 0.25;
			sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
			if (sensorReading >= steerConfig.PulseCountMax)
			{
				steerSwitch = 1; // reset values like it turned off
				currentState = 1;
				previous = 0;
			}
		}

		// Current sensor?
		if (steerConfig.CurrentSensor)
		{
      sensorReading = KeyaCurrentSensorReading;
      if (KeyaCurrentSensorReading >= steerConfig.PulseCountMax) {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
        engageCAN = false;
      }
		}

    switchByte = 0;
    switchByte |= (steerSwitch << 2);
		switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
		switchByte |= workSwitch;

    // =================================================================
    // CALCUL ANGLE BRAQUAGE
    //   IsDanfoss = 1 -> encodeur Keya + auto-zero
    //   IsDanfoss = 0 -> WAS physique ADS1115
    // =================================================================
    if (steerConfig.IsDanfoss)
    {
      // --- MODE HWT101CT + TM171 VIRTUAL WAS ---
      // HWT jest na zwrotnicy. Odejmujemy yaw calego ciagnika z TM171.
      // emaGpsHdg jest zapisany jako deg x10.
      HWTwasUpdate(TM171getYawDeg(), gpsSpeed, wasAogSeen && gpsSpeedUpdateTimer < 500 && !(guidanceStatus & 1));

      float rawAngle = HWTwasGetAngle();

      if (steerConfig.InvertWAS) rawAngle = -rawAngle;

      steerAngleActual   = rawAngle;
      helloSteerPosition = (int16_t)(rawAngle * 100.0f);
      steeringPosition   = helloSteerPosition;

      wasZeroDone = HWTwasReady();

      // Nie pozwalaj na autosterowanie przed pierwszym pewnym zerem
      // ani po utracie danych HWT/TM171.
      if (!HWTwasCanSteer()) watchdogTimer = WATCHDOG_FORCE_VALUE;
    }
    else
    {
      HWTinvalidate();
      wasZeroDone = false;
      if (!wasAdcAvailable) { watchdogTimer = WATCHDOG_FORCE_VALUE; steerAngleActual = 0; helloSteerPosition = 0; }
      else {
      // --- MODE WAS PHYSIQUE ADS1115 ---
      if (steerConfig.SingleInputWAS)
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();
      }
      else
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();
      }
      steeringPosition = (steeringPosition >> 1);
      helloSteerPosition = steeringPosition - 6800;

      if (steerConfig.InvertWAS)
      {
        steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);
        steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
      }
      else
      {
        steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
      }
    }

    }
    // Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    // HWT101CT auto-zero jest realizowane w HWT101CT_WAS.ino
    // =================================================================

		if (watchdogTimer < WATCHDOG_THRESHOLD && (steerConfig.IsDanfoss ? HWTwasCanSteer() : wasAdcAvailable))
  		{
  			steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
  			//if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;
  
  			calcSteeringPID();  //do the pid
  			motorDrive();       //out to motors the pwm value
  		}
		else
  		{
  			pwmDrive = 0; //turn off steering motor
  			disableKeyaSteer(); // If we lost the connection to AOG, definitely disable steering
  			motorDrive(); //out to motors the pwm value
  			pulseCount = 0;
  		}
     if (Brand == 3) SetRelaysFendt();
	} //end of timed loop

	//This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
	//delay(1);

	// Speed pulse
if (gpsSpeedUpdateTimer < 1000)
{
    if (speedPulseUpdateTimer > 200)
    {
        speedPulseUpdateTimer = 0;

        // gpsSpeed en km/h → m/s = /3.6 → * imp/m = fréquence Hz
        float speedPulse = (gpsSpeed / 3.6f) * SPEED_PULSE_IMP_PER_METER;

        if (gpsSpeed > 0.11) {
            tone(velocityPWM_Pin, uint16_t(speedPulse));
        }
        else {
            noTone(velocityPWM_Pin);
        }
    }
}


  if (sectionout == 1)
      {
        sectionControlLoop(); // ajout FlorianT
      }


} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp()
{
	// When ethernet is not running, return directly. parsePacket() will block when we don't
	if (!Ethernet_running)
	{
		return;
	}

	uint16_t len = Eth_udpAutoSteer.parsePacket();

	// if (len > 0)
	// {
	//  Serial.print("ReceiveUdp: ");
	//  Serial.println(len);
	// }

	// Check for len > 4, because we check byte 0, 1, 3 and 3
	if (len > 4)
	{
		Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

		if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
		{
			if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
			{
				gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
				gpsSpeedUpdateTimer = 0;
        wasAogSeen = true;

				prevGuidanceStatus = guidanceStatus;

				guidanceStatus = autoSteerUdpData[7];
				guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

				//Bit 8,9    set point steer angle * 100 is sent
				steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; //high low bytes

				//Serial.print("steerAngleSetPoint: ");
				//Serial.println(steerAngleSetPoint);

				//Serial.println(gpsSpeed);

				if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1) || (steerConfig.IsDanfoss ? !HWTwasCanSteer() : !wasAdcAvailable))
				{
					watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
				}
				else          //valid conditions to turn on autosteer
				{
					watchdogTimer = 0;  //reset watchdog
				}

				//----------------------------------------------------------------------------
				//Serial Send to agopenGPS

				int16_t sa = (int16_t)(steerAngleActual * 100);

				PGN_253[5] = (uint8_t)sa;
				PGN_253[6] = sa >> 8;

				// heading et roll : valeurs sentinelles 9999/8888
				// Le BNO de ce sketch est envoye via helloFromIMU (PGN 121)
				// et non via PGN 253 – comportement identique a l'original
				PGN_253[7] = (uint8_t)9999;
				PGN_253[8] = 9999 >> 8;
				PGN_253[9]  = (uint8_t)8888;
				PGN_253[10] = 8888 >> 8;

				PGN_253[11] = switchByte;
				PGN_253[12] = (uint8_t)pwmDisplay;

				//checksum
				int16_t CK_A = 0;
				for (uint8_t i = 2; i < PGN_253_Size; i++)
					CK_A = (CK_A + PGN_253[i]);

				PGN_253[PGN_253_Size] = CK_A;

				//off to AOG
				SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

				//Steer Data 2 -------------------------------------------------
				if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
				{
					if (aog2Count++ > 2)
					{
						//Send fromAutosteer2
						PGN_250[5] = (byte)sensorReading;

						//add the checksum for AOG2
						CK_A = 0;

						for (uint8_t i = 2; i < PGN_250_Size; i++)
						{
							CK_A = (CK_A + PGN_250[i]);
						}

						PGN_250[PGN_250_Size] = CK_A;

						//off to AOG
						SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
						aog2Count = 0;
					}
				}

				//Serial.println(steerAngleActual);
				//--------------------------------------------------------------------------
			}

			//steer settings
			else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
			{
				//PID values
				steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

				steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

				steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

				steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

				float temp = (float)steerSettings.minPWM * 1.2;
				steerSettings.lowPWM = (byte)temp;

				steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

        // W trybie HWT steerSensorCounts nie jest uzywany do pomiaru kata.


				steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

				steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi

				steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

				//crc
				//autoSteerUdpData[13];

				//store in EEPROM
				EEPROM.put(10, steerSettings);

        // Zero HWT jest ustalane automatycznie po stabilnej jezdzie prosto.


				// Re-Init steer settings
				//steerSettingsInit();
			}

			else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
			{
        uint8_t oldMode = steerConfig.IsDanfoss, oldInvert = steerConfig.InvertWAS;
				uint8_t sett = autoSteerUdpData[5]; //setting0

				if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
				if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
				if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
				if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
				if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
				if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
				if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
				if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

				steerConfig.PulseCountMax = autoSteerUdpData[6];

				//was speed
				//autoSteerUdpData[7];

				sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

				if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
				if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
				if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
				if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

				//crc
				//autoSteerUdpData[13];

        if (oldMode != steerConfig.IsDanfoss || oldInvert != steerConfig.InvertWAS) {
          HWTinvalidate(); watchdogTimer = WATCHDOG_FORCE_VALUE; pwmDrive = 0; motorDrive();
        }
				EEPROM.put(40, steerConfig);


			}//end FB
			else if (autoSteerUdpData[3] == 200) // Hello from AgIO
			{
				if (Autosteer_running)
				{
					int16_t sa = (int16_t)(steerAngleActual * 100);

					helloFromAutoSteer[5] = (uint8_t)sa;
					helloFromAutoSteer[6] = sa >> 8;

					helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
					helloFromAutoSteer[8] = helloSteerPosition >> 8;
					helloFromAutoSteer[9] = switchByte;

					SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
				}
				if (useBNO08x || useTM171)
				{
					SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
				}
          //SendUdp(helloFromMachine, sizeof(helloFromMachine), Eth_ipDestination, portDestination);
          //sendHelloToAgIO();
        helloFromMachine[5] = relay;
        helloFromMachine[6] = relayHi;
    
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(helloFromMachine) - 1; i++)
          {
              CK_A = (CK_A + helloFromMachine[i]);
          }

        if (sectionout == 1)
          {
            helloFromMachine[sizeof(helloFromMachine) - 1] = CK_A;
            SendUdp(helloFromMachine, sizeof(helloFromMachine), Eth_ipDestination, portDestination);
          }}

			else if (autoSteerUdpData[3] == 201)
			{
				//make really sure this is the subnet pgn
				if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
				{
					networkAddress.ipOne = autoSteerUdpData[7];
					networkAddress.ipTwo = autoSteerUdpData[8];
					networkAddress.ipThree = autoSteerUdpData[9];

					//save in EEPROM and restart
					EEPROM.put(60, networkAddress);
					SCB_AIRCR = 0x05FA0004; //Teensy Reset
				}
			}//end 201

			//whoami
			else if (autoSteerUdpData[3] == 202)
			{
				//make really sure this is the reply pgn
				if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
				{
					IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

					//hello from AgIO
					uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
						Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3],
						rem_ip[0],rem_ip[1],rem_ip[2], 23 };

					//checksum
					int16_t CK_A = 0;
					for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
					{
						CK_A = (CK_A + scanReply[i]);
					}
					scanReply[sizeof(scanReply) - 1] = CK_A;

					static uint8_t ipDest[] = { 255,255,255,255 };
					uint16_t portDest = 9999; //AOG port that listens

					//off to AOG
					SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
				}
			}
       else if (autoSteerUdpData[3] == 236) // Relay Pin Settings ajouté par FlorianT
            {
                updatePinMapping(autoSteerUdpData);
            }
      else if (autoSteerUdpData[3] == 238)
            {
                aogConfig.raiseTime = autoSteerUdpData[5];
                aogConfig.lowerTime = autoSteerUdpData[6];
                //aogConfig.enableToolLift = autoSteerUdpData[7];

                //set1 
                uint8_t sett = autoSteerUdpData[8];  //setting0     
                if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;
                if (bitRead(sett,1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;  

                //crc

                //save in EEPROM and restart
                EEPROM.put(70, aogConfig);
                //resetFunc();
                isHeaderFound = isPGNFound = false;
                pgn=dataLength=0;
            }
      else if (autoSteerUdpData[3] == 0xEF && Autosteer_running)  //239
      {
        //digitalWrite(PWM2_RPWM, HIGH);
                handlePGN239(autoSteerUdpData);  // Tout est géré dans SectionControl.cpp
        //reset for next pgn sentence
        isHeaderFound = isPGNFound = false;
        pgn = dataLength = 0;
        watchdogKeya = 0;
      }
		} //end if 80 81 7F
	}
}
#endif

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t* data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
	Eth_udpAutoSteer.beginPacket(dip, dport);
	Eth_udpAutoSteer.write(data, datalen);
	Eth_udpAutoSteer.endPacket();
}
#endif

//ISR Steering Wheel Encoder
void EncoderFunc()
{
	if (encEnable)
	{
		pulseCount++;
		encEnable = false;
	}
}

//Hitch Control------------------------------------------------------------
void SetRelaysFendt(void)
{
  uint32_t currentMillis = millis();


  if (currentMillis - lastpushbutton >= 300) {
      if (goDown)  liftGo();  
      if (endDown) liftEnd();
  }

//If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
  if (aogConfig.isRelayActiveHigh == 1){
    bitState = (bitRead(relay, 0));
  }
//If not selected hitch command is used on headland used as Trigger  
  else{
    if (hydLift == 1) {bitState = 1;}
    if (hydLift == 2) {bitState = 0;}
}
if (aogConfig.enableToolLift == 1){
  if (bitState  && !bitStateOld) 
    {
      pressGo();
      lastpushbutton = currentMillis;
    }
  if (!bitState && bitStateOld) 
  {
    pressEnd(); //Press End button - CAN Page
   lastpushbutton = currentMillis;
  }
}
bitStateOld = bitState;
}
