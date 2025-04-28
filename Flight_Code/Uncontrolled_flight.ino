#include "I2Cdev.h"
#include <ESP32Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <SD.h>
#include <FS.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define BNO08X_INT  -1
#define BNO08X_RST  -1
#define BNO08X_ADDR 0x4A 

#define INTERRUPT_PIN 2
#define LED_PIN 13
// Define the chip select pin for the SD card module
#define SD_CHIP_SELECT_PIN 5
#define buzzerPin 35
#define MOSFET_PIN1 25

Adafruit_BMP085 bmp;

BNO08x myIMU;

// Define two I2C buses
TwoWire I2C_1 = TwoWire(0);  // SDA=21, SCL=22
TwoWire I2C_2 = TwoWire(1);  // SDA=33, SCL=32

bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// PID variables
float setpointPitch = 0.0;
float inputPitch;
float outputPitch;
float errorPitch;
float lastErrorPitch = 0;
float integralPitch = 0;

float LoopTimer;
float currentTime = 9000;
float pidPitch;
float Roll=0;

float Rollcal = 0;
float Rollcalf = 0;
float lastRoll, outputRoll, lastErrorRoll, integralRoll, setpointRoll = 0, errorRoll, az1, ay1, zf, yf, ax1, xf;
int i, sample = 0;

String dataString = "";

// PID constants
float Kp = 0.5, Ki = 0.125, Kd = 0.2;

Servo servo1, servo3, servo2, servo4;
float pos1, pos2, pos3, pos4;

File dataFile; // File object for SD card

volatile bool mpuInterrupt = false;
bool timerActivated = false;

unsigned long startTime; // Variable to store the start time

// For Launch detection
const float ACC_THRESHOLD = 16.0;  // Threshold acceleration in m/s² to detect launch
const int SAMPLE_COUNT = 1;       // Number of consistent samples for detection for inal we keep it 5
float ax_mps2, ay_mps2, az_mps2;   // Acceleration in m/s²
int consistentLaunchReadings = 0;
bool launched = false;
float initialAltitude = 0.0; 
double altitude;
int16_t x, y, z; // Accelerometer raw values
unsigned long launchTime = NAN; // Time when the launch is detected
unsigned long buzzerStartTime = 0;

// Altitude and ignition variables
float currentAltitude = 0.0, maxAltitude = 0.0;
bool DrogueEjection = false;
bool DrogueCompleted = false;
bool MainEjection = false;
bool MainCompleted = false;
unsigned long DrogueEjectionTime = NAN;
unsigned long MainEjectionTime = NAN;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setServoPositions(float pos1, float pos2, float pos3, float pos4) {
    servo2.write(pos2);
    servo4.write(pos4);
    servo3.write(pos3);
    servo1.write(pos1);
}

void setReportsa(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableAccelerometer() == true) {
    Serial.println(F("Accelerometer enabled"));
    Serial.println(F("Output in form x, y, z, in m/s^2"));
  } else {
    Serial.println("Could not enable accelerometer");
  }
}

void detectLaunch(){

 // Serial.println("Launch Detect Loop");

  double altitude = bmp.readAltitude();

    if (!launched) {
        if (currentAltitude - initialAltitude >= 3) {
            consistentLaunchReadings++;
        } else {
            consistentLaunchReadings = 0;  
        }

        if (consistentLaunchReadings >= SAMPLE_COUNT) {
            // Launch detected
            launched = true;
            launchTime = millis();
            digitalWrite(buzzerPin, HIGH);  
            buzzerStartTime = millis();
           Serial.println("LAUNCH DETECTED!");
           delay(2000);
         dataString += String("\t LAUNCH DETECTED at: ") + String (launchTime/1000);
        }
    }
    if (launched && millis() - buzzerStartTime >= 1000){
//      Serial.println("Launch already detected. Continuing execution...");
      digitalWrite(buzzerPin, LOW);
    }
}


// Function to initialize SD card
void initSDCard() {
    Serial.print("Initializing SD card...");

    if (!SD.begin(SD_CHIP_SELECT_PIN)) {
        Serial.println("Card Mount Failed");
        return;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
  
    dataFile = SD.open("/Pitch_range_act.txt", FILE_WRITE);
    if (!dataFile) {
        Serial.println("Error opening data file!");
    } else {
        Serial.println("SD card initialized and data file opened successfully.");
    }
}

// Function to log data to SD card
void logDataToSDCard(String dataString) {
    if (!dataFile) {
        Serial.println("Error opening data file!");
        return;
    }

    float compare = millis() - startTime;

    if (compare >= currentTime) {
    if (dataFile) {
    dataFile.println(dataString);
    dataFile.flush();
    dataString = "";
  }

 else {
    Serial.println("Error opening datalog.txt");
      }
    currentTime += 4000;
    }
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, Roll, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
 }

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  // Initialize first I2C bus on pins (SDA=21, SCL=22)
    // Initialize first I2C bus for BNO085
    I2C_1.begin(21, 22);
    I2C_1.setClock(400000);  // Fast mode (400kHz)

    // Initialize second I2C bus for BMP180
    I2C_2.begin(33, 32);
    I2C_2.setClock(400000);  // Fast mode (400kHz)
    #endif

    Serial.begin(115200);
    while (!Serial);
    pinMode(MOSFET_PIN1, OUTPUT);
    delay(5000);
    pinMode(buzzerPin, OUTPUT);
    delay(500);
    digitalWrite(buzzerPin, HIGH);
    delay(400);
    digitalWrite(buzzerPin, HIGH);
    delay(400);
    digitalWrite(buzzerPin, LOW);


    Serial.println(F("BMP180 test"));

    bool status;

    status = bmp.begin(0x77, &I2C_2);  
  if (!status) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delay(1000);

  Serial.println();

    Serial.println(F("Initializing I2C devices..."));
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
      //if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (myIMU.begin(BNO08X_ADDR, I2C_1, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");
  setReports();

for (i = 0; i < 10; i++) {
    delay(50);
    if (myIMU.getSensorEvent() == true){
     if(myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
            Rollcal += ((myIMU.getYaw()) * 180.0 / PI); 
            Serial.print("Loop iteration: ");
            Serial.print(i);
            Serial.print(" Roll Calibration: ");
            Serial.println(Rollcal, 2);
            delay(100);
        } 
    }
            else {
            Serial.println("Sensor data not available, retrying...");
            delay(50); 
        }
  } 
    
      Rollcalf = Rollcal/10;
    //   xf = ax1/20.0;
    //   yf = ay1/20.0;
    //   zf = 9.8 - az1/20.0;

      delay(1000);

      Serial.print("Roll Error: ");
      Serial.println(Rollcalf);

  if (!bmp.begin(0x77, &I2C_2)) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1); 
  }

    initialAltitude = bmp.readAltitude();
    if (isnan(initialAltitude)) {
        Serial.println("BMP180 initialization failed!");
    }

    // servo1.attach(26);
    // servo2.attach(27);
    // servo3.attach(14);
    // servo4.attach(12);

    // servo2.write(90);
    // delay(15);
    // servo4.write(90);
    // delay(15);
    // servo1.write(90);
    // delay(15);
    // servo3.write(90);
    // delay(15);            

    Serial.println("Servos initialized at 90 degrees!");                
    Serial.println("System ready.");

    // Initialize SD card
    initSDCard();

     digitalWrite(buzzerPin, HIGH);
    delay(800);
    digitalWrite(buzzerPin, LOW);

    LoopTimer = millis();
    startTime = millis(); // Initialize the start time
}

void computePIDRoll(float dt, float Roll) {

    if (-90 < Roll && Roll <= 90) {
        setpointRoll = 0;
        } 
        else if (90 < Roll && Roll <= 179) {
            setpointRoll = 120;
        } 
        else if (-180 < Roll && Roll <= -90) {
            setpointRoll = -120;
        } 

    errorRoll = setpointRoll - Roll;

    integralRoll += errorRoll * dt;

    float derivativeRoll = (errorRoll - lastErrorRoll) / dt;

    outputRoll = Kp * errorRoll + Ki * integralRoll + Kd * derivativeRoll;

    pos1 = 90 - outputRoll;
    pos2 = 90 - outputRoll;
    pos3 = 90 - outputRoll;
    pos4 = 90 - outputRoll;

    
    bool clampedRoll = false;
    if (pos1 < 50 || pos1 > 130) {
        clampedRoll = true;
        pos1 = constrain(pos1, 50, 130);

    }
    if (pos3 < 50 || pos3 > 130) {
        clampedRoll = true;
        pos3 = constrain(pos3, 50, 130);
    }

    if (pos2 < 50 || pos2 > 130) {
        clampedRoll = true;
        pos2 = constrain(pos2, 50, 130);

    }
    if (pos4 < 50 || pos4 > 130) {
        clampedRoll = true;
        pos4 = constrain(pos4, 50, 130);
    }

    // Serial.print("Pos: ");
    // Serial.println(pos1);
    // Serial.print("Deflection Angle: ");
    // Serial.println(outputRoll);
    // Serial.print("Setpoint: ");
    // Serial.println(setpointRoll);

 //   setServoPositions(pos1, pos2, pos3, pos4);

    if (clampedRoll) {
        integralRoll -= errorRoll * dt;
    }

    lastErrorRoll = errorRoll;
    lastRoll = Roll;
}

void TriggerDrogueEjection() {
    Serial.println("MOSFET HIGH");
   digitalWrite(MOSFET_PIN1, HIGH); 
    dataString += String("\t M ON");
    DrogueEjection = true;
    DrogueEjectionTime = millis();
}

void CheckDrogueEjectionDuration() {
    if (millis() - DrogueEjectionTime >= 4000) { 
        digitalWrite(MOSFET_PIN1, LOW); 
        dataString += String("\t M OFF");
        Serial.println("Drogue Parachute Ejection Completed: MOSFET1 OFF");
        DrogueEjection = false;
        DrogueCompleted = true;
    }
}

void loop() {

    if (myIMU.wasReset()) {
    Serial.print("Sensor was reset ");
    setReports();
     }

    bool gotRotation = false;
    bool gotAccel = false;

    if (myIMU.getSensorEvent() == true) {  // Fetch the latest IMU event

    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

      Roll = (myIMU.getYaw()) * 180.0 / PI - Rollcalf;
    //  Serial.println(Roll);
      gotRotation = true; 

  //  Serial.print("Updated Roll: ");
    Serial.println(Roll);
    }
  }

    detectLaunch();

        // Get current timestamp and compute dt
        unsigned long currentMillis = millis();
        float dt = (currentMillis - LoopTimer) / 1000.0;
        LoopTimer = currentMillis;
        float time = millis() - startTime;

        // PID calculations
        computePIDRoll(dt, Roll);

        // Serial.print("Loopcount:");
        // Serial.print("\t");
        // Serial.print(LoopTimer);
        // Serial.print("\t");
        // Serial.print(":");
        // Serial.print(outputRoll);
        // Serial.print("\t");
        Serial.println(Roll);
        
        dataString += String("\n") + String(time) + "," + String(outputRoll) + "," + String(Roll) + String("\n");
        logDataToSDCard(dataString);

    double altitude = bmp.readAltitude();

       if (!isnan(altitude)) {
        currentAltitude = altitude;

        if (currentAltitude > maxAltitude) {
            maxAltitude = currentAltitude;
        }

        // Check for ignition trigger
        if (!DrogueEjection && !DrogueCompleted && (maxAltitude - currentAltitude) > 3.5 && launched) {
            Serial.print("Drougue Ejection Triggered! at  ");
            Serial.println(maxAltitude - currentAltitude);
            TriggerDrogueEjection();
        }

        // Check ignition duration
        if (DrogueEjection && !DrogueCompleted) {
          Serial.print("Duration Check Drougue Ejection!");
            CheckDrogueEjectionDuration();
        }

    } else {
        Serial.println("Failed to read altitude.");
    }

    if (launched && !timerActivated && (millis() - launchTime >= 9000)) { //needs to be changed last minute
        if (!DrogueEjection && !DrogueCompleted) {
            Serial.println("Timer activated 13 seconds after launch detection");
            dataString += String("\t Timer");
            TriggerDrogueEjection();
        }
        if (DrogueEjection && !DrogueCompleted) {
          CheckDrogueEjectionDuration();
          timerActivated = true; 
        }   
  }
        

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
}