#include <Arduino.h>
#include <Preferences.h>
#include <TMCStepper.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <string.h>
#include <sstream>

//------ flash memory ------
Preferences settings;
int resetToDefaults = 0;

//------ limits switches ------
#define LIMIT_START_PIN 19
#define LIMIT_END_PIN 18
void IRAM_ATTR onLimitStart();
void IRAM_ATTR onLimitEnd();
volatile int handleLimitStart = 0;
volatile int handleLimitEnd = 0;
portMUX_TYPE limitSynch = portMUX_INITIALIZER_UNLOCKED;

//------ stepper driver ------
#define RXD2 16             // HWUART recieve
#define TXD2 17             // HWUART transmit
#define EN_PIN 32           // Enable
#define DIR_PIN 25          // Direction
#define STEP_PIN 33         // Step
#define SERIAL_PORT Serial2 // HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f       // SilentStepStick sense resistor
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

// ------ movement and position ------
#define DEFAULT_STELLAR_SPEED 33.8892
#define INDEX_PIN 26   // Index
#define REVERSE true   // change based on experimental motor direction
#define FORWARDS false // change based on experimental motor direction
int8_t stepPinValue = 0;
unsigned long nextStandstillPulse = 0;
uint32_t stepsStart = 0;
uint16_t cycleCount = 0;
uint32_t stepsEnd = 0;
int32_t motorSpeed = 0;
int32_t maxSpeed = 30000;
int noblockAccel = 0;
void IRAM_ATTR onIndex();
portMUX_TYPE indexSynch = portMUX_INITIALIZER_UNLOCKED;
void blockingAccel(int32_t target, int16_t rate);
uint8_t tracking = 0;
double stellarSpeed;
double lunarSpeed;
uint32_t travel;
short center = 0;
short firstReset = 1;
class Position
{
private:
  unsigned long time;
  int32_t steps;

public:
  void getPosition()
  {
    time = millis();
    steps = (1024 * cycleCount) + driver.MSCNT();
  }
};
Position tuneStart;
Position tuneEnd;
int32_t getPosition();

//------ BLE ------
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define LED_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define MOVE_UUID "b5489b57-fd0c-4889-a942-861f39906a45"
#define RESET_UUID "27e5eb71-4b57-4a18-873c-aa272f770ca7"
#define TRACKING_UUID "1043dbcc-f3a0-4fa7-b77c-d7598a3868b4"
#define TUNE_START_UUID "e15d4ed0-d255-465c-a6af-33ce7b7ecd69"
#define TUNE_END_UUID "e36a814d-017e-4d96-99a1-3fd3d86fa194"
#define RESET_TO_DEFAULTS_UUID "818b22cf-6143-415f-8258-503688dc2a3a"
#define CENTER_UUID "f49e1703-0efe-46ee-8959-448041f96750"
bool deviceConnected = false;
bool oldDeviceConnected = false;
static bool led_on = false;
class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

//------ sound ------
#define BUZZER_PIN 23
#define BUZZER_CHANNEL 0
int song = 0;
int songCount = 0;
int noteIndex = 0;
unsigned long noteEnd = 0;
class Note
{
public:
  int duration;
  note_t note;
  int octave;
  Note(note_t nte, int octve, int duratn)
  {
    note = nte;
    octave = octve;
    duration = duratn;
  }
};
Note songOne[] = {Note(NOTE_C, 6, 500), Note(NOTE_G, 6, 500), Note(NOTE_C, 7, 500), Note(NOTE_E, 7, 200), Note(NOTE_Eb, 7, 500)};
Note songTwo[] = {Note(NOTE_C, 7, 100), Note(NOTE_D, 7, 100), Note(NOTE_E, 7, 100), Note(NOTE_F, 7, 100), Note(NOTE_G, 7, 100)};
Note songThree[] = {Note(NOTE_C, 7, 200), Note(NOTE_G, 6, 200), Note(NOTE_C, 7, 200), Note(NOTE_G, 6, 200), Note(NOTE_C, 7, 200), Note(NOTE_G, 6, 200), Note(NOTE_C, 7, 200), Note(NOTE_G, 6, 200)};
Note songFour[] = {Note(NOTE_G, 7, 100), Note(NOTE_F, 7, 100), Note(NOTE_E, 7, 100), Note(NOTE_D, 7, 100), Note(NOTE_C, 7, 100)};
Note *songs[] = {songOne, songTwo, songThree, songFour};
int songsLen[] = {5, 5, 8, 5};

void setup()
{
  Serial.begin(115200);

  //====== pins and interrupts ======
  //------ stepper driver ------
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);           // Disable driver in hardware until setup is complete
  digitalWrite(STEP_PIN, stepPinValue); // get a start for step pin
  pinMode(INDEX_PIN, INPUT);
  //------ limits ------
  pinMode(LIMIT_START_PIN, INPUT_PULLUP);
  pinMode(LIMIT_END_PIN, INPUT_PULLUP);
  attachInterrupt(LIMIT_START_PIN, onLimitStart, FALLING);
  attachInterrupt(LIMIT_END_PIN, onLimitEnd, FALLING);
  //------ movement and position ------
  pinMode(INDEX_PIN, INPUT);
  attachInterrupt(INDEX_PIN, onIndex, RISING);
  // for testing
  pinMode(LED_BUILTIN, OUTPUT);

  //====== load data from flash ======
  settings.begin("Settings", false);                                        // 1stArg = namespace, false = R/W access
  travel = settings.getUInt("travel", 0);                                   // load travel
  stellarSpeed = settings.getDouble("stellarSpeed", DEFAULT_STELLAR_SPEED); // load stellar speed, 2nd arg is default
  lunarSpeed = .96622929 * stellarSpeed;                                    // calc lunarSpeed

  //====== stepper driver setup ======
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // begin hardware Serial2
  driver.begin();                                // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.en_spreadCycle(false);                  // spreadCycle off
  driver.pwm_autoscale(true);                    // enable stealthChop
  driver.toff(5);                                // enable driver via software
  driver.rms_current(600, .7);                   // Set motor RMS current, the 1 sets holding current to equal run current (IHOLD/IRUN = 1)
  driver.microsteps(256);                        // Set microsteps to 1/256
  driver.I_scale_analog(false);                  // do no use external trimpot for current scaling
  driver.TPOWERDOWN(255);                        // ~5 sec before lowering to IHOLD current
  driver.iholddelay(15);                         // slow decay of IRUN to IHOLD
  driver.index_step(0);                          // pulse INDEX for every 1024 microsteps
  driver.VACTUAL(0);                             // make sure internal pulse generator is 0
  driver.shaft(false);                           // reverse direction
  digitalWrite(EN_PIN, LOW);                     // Enable driver in hardware
  //------ Automatic Tuning phase 1 ------
  Serial.println("Stepper driver tuning phase 1");
  for (uint16_t i = 100; i > 0; i--)
  {
    digitalWrite(STEP_PIN, stepPinValue);
    stepPinValue = 1 - stepPinValue;
    delayMicroseconds(320);
  }

  //====== BLE Server Setup ======
  Serial.println("Setting up BLE Server");
  // ------ setup device, server, and service ------
  BLEDevice::init("Astromods LED");
  BLEServer *pServer = BLEDevice::createServer();
  // Set server callbacks
  Serial.println("Setting up callbacks:");
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // ------ create characteristics and their callbacks ------
  Serial.println("LED");
  // LED characteristic
  BLECharacteristic *pCharacteristicLed = pService->createCharacteristic(
      LED_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristicLed->setValue("false");
  // LED callback
  class ledCallback : public BLECharacteristicCallbacks
  {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.println("Led detected");
      led_on = !led_on;
      digitalWrite(LED_BUILTIN, led_on);
    }
  };
  pCharacteristicLed->setCallbacks(new ledCallback());
  // move mount characteristic
  BLECharacteristic *pCharacteristicMove = pService->createCharacteristic(
      MOVE_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  // move mount callback
  class moveCallback : public BLECharacteristicCallbacks
  {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.println("Move mount detected");
      std::string sMove = pCharacteristic->getValue();
      int8_t move = sMove.c_str()[0];
      Serial.print("move = ");
      Serial.println(move);
      if ((move > 0) & !handleLimitEnd)
      {
        // TODO calc a better rate
        driver.VACTUAL((motorSpeed + 5000) / .715);
      }
      else if ((move < 0) & !handleLimitStart)
      {
        // TODO calc a better rate
        driver.VACTUAL((motorSpeed - 5000) / .715);
      }
      else
      {
        driver.VACTUAL(motorSpeed);
      }
    }
  };
  pCharacteristicMove->setCallbacks(new moveCallback());
  // reset characteristic
  BLECharacteristic *pCharacteristicRESET = pService->createCharacteristic(
      RESET_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  // reset callback
  class resetCallback : public BLECharacteristicCallbacks
  {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.print("Reset detected");
      if (digitalRead(LIMIT_START_PIN))
      {
        song = 3;
        delay(2000);
        noblockAccel = -30000;
        // blockingAccel(-30000, 500);
      }
    }
  };
  pCharacteristicRESET->setCallbacks(new resetCallback());
  // tuneStart characteristic
  BLECharacteristic *pCharacteristicTuneStart = pService->createCharacteristic(TUNE_START_UUID, BLECharacteristic::PROPERTY_READ);
  // tuneStart callback
  class tuneStartCallback : public BLECharacteristicCallbacks
  {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.println("Tune start detected");
      tuneStart.getPosition();
    }
  };
  pCharacteristicTuneStart->setCallbacks(new tuneStartCallback());
  // tuneEnd characteristic
  BLECharacteristic *pCharacteristicTuneEnd = pService->createCharacteristic(TUNE_END_UUID, BLECharacteristic::PROPERTY_READ);
  // tuneEnd callback
  class tuneEndCallback : public BLECharacteristicCallbacks
  {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.println("Tune end detected");
      tuneEnd.getPosition();
      // TODO calculate new rate based on tuneStart and distance between
    }
  };
  pCharacteristicTuneEnd->setCallbacks(new tuneEndCallback());
  // tracking characteristic
  BLECharacteristic *pCharacteristicTRACKING = pService->createCharacteristic(
      TRACKING_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  // tracking callback
  class trackingCallback : public BLECharacteristicCallbacks
  {
    void onRead(BLECharacteristic *pCharacteristic)
    {
      // std::ostringstream trackingStr;
      // trackingStr << tracking;
      // Serial.println("Tracking read detected, sending tracking as ");
      // Serial.println(trackingStr.str());
      char trackingStr[1];
      itoa(tracking, trackingStr, 10);
      Serial.println("Tracking read detected, sending tracking as ");
      Serial.println(trackingStr);
      pCharacteristic->setValue(trackingStr); // start with tracking off
    }
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.println("Tracking detected");
      std::string sTracking = pCharacteristic->getValue();
      tracking = sTracking.c_str()[0];
      Serial.print("Tracking = ");
      Serial.println(tracking);
      if (tracking == 1)
      {
        Serial.println("tracking set to stellar");
        blockingAccel(stellarSpeed, 100); // stellar tracking
      }
      else if (tracking == 2)
      {
        Serial.println("tracking set to lunar");
        blockingAccel(lunarSpeed, 100); // lunar tracking
      }
      else
      {
        Serial.println("tracking set to off");
        blockingAccel(0, 500); // stop
      }
    }
  };
  pCharacteristicTRACKING->setCallbacks(new trackingCallback());
  // reset_to_defaults characteristic
  BLECharacteristic *pCharacteristicRESET_TO_DEFAULTS = pService->createCharacteristic(
      RESET_TO_DEFAULTS_UUID, BLECharacteristic::PROPERTY_WRITE);
  // reset_to_defaults callback
  class resetToDefaultsCallback : public BLECharacteristicCallbacks
  {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.print("Reset_to_defaults detected");
      resetToDefaults = 1; // flag for reset
    }
  };
  pCharacteristicRESET_TO_DEFAULTS->setCallbacks(new resetToDefaultsCallback());
  // center characteristic
  BLECharacteristic *pCharacteristicCENTER = pService->createCharacteristic(
      CENTER_UUID, BLECharacteristic::PROPERTY_WRITE);
  // center callback
  class centerCallback : public BLECharacteristicCallbacks
  {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      Serial.print("Center detected");
      center = 1; // flag for centering
    }
  };
  pCharacteristicCENTER->setCallbacks(new centerCallback());

  // ------ start BLE ------
  Serial.println("Starting BLE");
  pService->start();
  Serial.println("Start advertising");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  // sound
  song = 1;
  Serial.print("vactual is ");
  Serial.println(driver.VACTUAL());
}

void loop()
{
  // ------ handle first reset ------
  if (firstReset)
  {
    // TODO
    firstReset = 0; // flag as handled
  }

  // ------ handle centering ------
  if (center)
  {
  }

  // ------ maintain run current and AT phase #1 during standstill ------
  if (motorSpeed == 0)
  {
    unsigned long time = millis();
    if (time > nextStandstillPulse)
    {
      digitalWrite(STEP_PIN, stepPinValue);
      stepPinValue = 1 - stepPinValue;
      nextStandstillPulse = time + 500;
    }
  }

  // ------ non-blocking acceleration ------
  if (noblockAccel)
  {
    Serial.print("noblockAccel is ");
    Serial.println(noblockAccel);
    if (motorSpeed > noblockAccel)
    {
      Serial.println("motorSpeed > noblockAccel");
      if (motorSpeed - noblockAccel < 300)
      {
        motorSpeed = noblockAccel;
        noblockAccel = 0;
        driver.VACTUAL(motorSpeed);
        Serial.print("speed changed to ");
        Serial.println(motorSpeed);
      }
      else
      {
        motorSpeed -= 300;
        driver.VACTUAL(motorSpeed);
        Serial.print("speed changed to ");
        Serial.println(motorSpeed);
      }
    }
    else if (motorSpeed < noblockAccel)
    {
      if (noblockAccel - motorSpeed > 300)
      {
        motorSpeed = noblockAccel;
        noblockAccel = 0;
        driver.VACTUAL(motorSpeed);
      }
      else
      {
        motorSpeed += 300;
        driver.VACTUAL(motorSpeed);
      }
    }
    else
    {
      noblockAccel = 0; // motorSpeed = target, no acceleration
    }
  }

  // ------ limits ------
  if (handleLimitStart)
  {
    if (motorSpeed < 0)
    {
      Serial.println("handleLimitStart");
      blockingAccel(0, 2000);      // aggressive stop
      noblockAccel = 0;            // stop any acceleration
      cycleCount = 0;              // reset cycleCount
      stepsStart = driver.MSCNT(); // reset stepsStart
      handleLimitStart = 0;        // limitStart has been handled
      if (tracking == 1)
      {

        blockingAccel(stellarSpeed, 100);
      }
      else if (tracking == 2)
      {
        blockingAccel(lunarSpeed, 100);
      }
      song = 2;
    }
    else
    {
      handleLimitStart = 0; // ignore
    }
  }
  else if (handleLimitEnd)
  {
    if (motorSpeed > 0)
    {
      Serial.println("handleLimitEnd");
      blockingAccel(0, 2000); // aggressive stop
      noblockAccel = 0;       // stop any acceleration
      stepsEnd = getPosition();
      int32_t latestTravel = stepsEnd - stepsStart;
      if (latestTravel - travel > 100) // if much different from old travel then...
      {
        travel = latestTravel;
        Serial.print("New travel calculated to be");
        Serial.println(travel);
        settings.putUInt("travel", travel); // ...update travel
      }

      song = 4;           // sound
      handleLimitEnd = 0; // limitEnd has been handled
    }
    else
    {
      handleLimitEnd = 0; // ignore
    }
  }

  // ------ sound ------
  if (song)
  {
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL); // TODO try reattaching after song end
    if (millis() > noteEnd)
    {
      if (noteIndex < songsLen[song - 1])
      {
        Note note = songs[(song - 1)][noteIndex];
        ledcWriteNote(BUZZER_CHANNEL, note.note, note.octave);
        noteEnd = millis() + note.duration;
        noteIndex++;
      }
      else
      {
        ledcDetachPin(BUZZER_PIN);
        song = 0;
        noteIndex = 0;
      }
    }
  }

  if (resetToDefaults)
  {
    settings.putDouble("stellarSpeed", DEFAULT_STELLAR_SPEED);
    stellarSpeed = DEFAULT_STELLAR_SPEED;
    lunarSpeed = .96622929 * stellarSpeed; // calc lunarSpeed;
  }
}

//====== ISRs ======

void IRAM_ATTR onLimitStart()
{
  portENTER_CRITICAL(&limitSynch);
  handleLimitStart = 1;
  portEXIT_CRITICAL(&limitSynch);
}

void IRAM_ATTR onLimitEnd()
{
  portENTER_CRITICAL(&limitSynch);
  handleLimitEnd = 1;
  portEXIT_CRITICAL(&limitSynch);
}

void IRAM_ATTR onIndex()
{
  portENTER_CRITICAL(&indexSynch);
  cycleCount++;
  portEXIT_CRITICAL(&indexSynch);
}

// void IRAM_ATTR onLimitEndRising()
// {
//   portENTER_CRITICAL(&limitSynch);
//   ignoreLimitEnd = 1;
//   portEXIT_CRITICAL(&limitSynch);
// }

// void IRAM_ATTR onLimitStartRising()
// {
//   portENTER_CRITICAL(&limitSynch);
//   ignoreLimitStart = 1;
//   portEXIT_CRITICAL(&limitSynch);
// }

//------ position and movement ------
void blockingAccel(int32_t target, int16_t rate)
{
  if (target < motorSpeed)
  {
    rate *= -1;
  }
  int initial = (target - motorSpeed) % rate;
  motorSpeed += initial;
  Serial.print("Motor speed accelerated to ");
  Serial.println(motorSpeed);
  driver.VACTUAL(motorSpeed / .715);
  while (motorSpeed != target)
  {
    motorSpeed += rate;
    Serial.print("Motor speed accelerated to ");
    Serial.println(motorSpeed);
    driver.VACTUAL(motorSpeed / .715);
  }
}

int32_t getPosition()
{
  int16_t current = driver.MSCNT();
  int32_t position = (1024 * cycleCount) + current;
  return position;
}

void goTo(int32_t to)
{
  int from = getPosition();
  if (from < to)
  {
    int distance = to - from;
    int halfway = from + distance / 2;
    while (getPosition() < halfway && motorSpeed + 300 < maxSpeed)
    {
      motorSpeed += 300; // accelerate
      driver.VACTUAL(motorSpeed);
    }
    int beginStop = to - (getPosition() - from);
    while (getPosition() < beginStop)
    {
      // wait until needing to slow
    }
    while (motorSpeed - 300 > 0)
    {
      motorSpeed -= 300;
      driver.VACTUAL(motorSpeed); // decelerate
    }
    driver.VACTUAL(0); // stop
  }
  else if (from > to)
  {
    int distance = from - to;
    int halfway = from - distance / 2;
    while (getPosition() > halfway && motorSpeed - 300 > maxSpeed * -1)
    {
      motorSpeed -= 300; // accelerate
      driver.VACTUAL(motorSpeed);
    }
    int beginStop = to + (from - getPosition());
    while (getPosition() > beginStop)
    {
      // wait until needing to slow
    }
    while (motorSpeed + 300 < 0)
    {
      motorSpeed += 300;
      driver.VACTUAL(motorSpeed); // decelerate
    }
    driver.VACTUAL(0); // stop
  }
  from = getPosition();
  if (from > to)
  {
    driver.shaft(REVERSE); // change direction
  }
  while (getPosition() != to)
  {
    digitalWrite(STEP_PIN, stepPinValue); // crawl to position
    stepPinValue = 1 - stepPinValue;
    delay(50);
  }
  driver.shaft(FORWARDS); // reset direction to normal
}
