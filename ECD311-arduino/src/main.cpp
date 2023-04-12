// Libraries
#include <ACS712.h>  // Library for current sensors

#include <Arduino.h>
#include <DS3231_Simple.h>    // Library for Real Time Clock
//Tracking
#include <SPI.h>              // Library for communication (may be unused)
#include <SparkFunLSM9DS1.h>  // Library for accelerometer and compass
#include <Wire.h>             // Library for I2C communication
#include <Adafruit_SHT31.h>  // Library for Temp and Humidity Sensor(SHT30 is compatible)

// Non-Pin Setup for Components
// Pure Inputs
/* //Tracking
// Temperature/Humidity Sensor
Adafruit_SHT31 sht30 = Adafruit_SHT31();
// Compass+accelerometer
LSM9DS1 Comp;
LSM9DS1 Accel;
// I2C addresses for compass and accelerometer communication
#define LSM9DS1_C_C 0x1E  // Comp's compass
#define LSM9DS1_A_C 0x6B  // Comp's accelerometer
#define LSM9DS1_C_A 0x1C  // Accel's compass
#define LSM9DS1_A_A 0x6A  // Accel's accelerometer
*/
// Real Time Clock
DS3231_Simple Clock;
DateTime MyDateAndTime;
DateTime Today_Sunrise;
DateTime Today_Sunset;
// Analog Pins
ACS712 SP_CUR(A0);     // Solar Panel current sensor
ACS712 BATT_CUR(A1);   // Battery current sensor
ACS712 LOAD_CUR(A2);   // Load current sensor
int SP_VOLT = A7;      // Voltage of Solar Panel
int BATT24_VOLT = A6;  // Voltage of both batteries
int BATT12_VOLT = A5;  // Voltage of 1st batteries
int W_SIG = A15;       // Wind Meter (Anenometer)
// Digital Pins
// Manual Controls for both Motors
int MANUAL = 19;  // Activates Motor Manual mode
int M2_EAST = 18;
int M2_WEST = 17;
int M1_UP = 16;
int M1_DN = 15;
// Motor Driver Inputs
int M1_PUL = 13;
int M1_DIR = 12;
int M2_PUL = 4;
int M2_DIR = 3;
/* //Tracking
int LIMIT_SIG_1 = 9;  // Limit Switch 1(CHECK if elev or Azi)
int LIMIT_SIG_2 = 8;  // Limit Switch 2(CHECK if elev or Azi)
*/
// Other
int InverterEnable = 11;
// Global Variables
// Sensors
double PanelCurrent = 0;
double BatteryCurrent = 0;
double LoadCurrent = 0;
double BatteryTotalVoltage = 0;
double PanelVoltage = 0;
double BatteryOneVoltage = 0;
double WindSpeed = 0;
 //Tracking
double MeasuredAzimuth = 0.0;    // AKA Yaw or Heading
double MeasuredElevation = 0.0;  // AKA Zenith or Pitch
double MeasuredRoll = 0.0;       // Unused, solar panels don't roll
double Temp = 0;
double Humid = 0;

// Motors
int M1Running = 0;
int M2Running = 0;
/* //Tracking
//Adds buffer for the motor movement to prevent jittering
int AziSpace = 0;   
int ElevSpace = 0;
*/
// Finite State Machine
int State = 0;
/* //Tracking
int WindyWeather = 0;
int SnowyWeather = 0;
int CloudyWeather = 0;
int NEAR_LIM1 = 0;
int NEAR_LIM2 = 0;
*/
// Data Transfer
int PiComm = 0;
 //Tracking
int SunriseAzimuth = 0;
int SunriseElevation = 0;
int morning_lockout = 0;
int AzimuthCommand = 0;
int ElevationCommand = 0;

// Definitions
/* //Tracking
#define ElevRange 5.0
#define AziRange 5.0
#define SnowElev 80.0
#define SnowAzi 180.0 
#define LimitElev 0.0 //CHECK
#define LimitAzi 0.0 //CHECK, may not need
#define WindElev 0.0
//Average position of the sun in Bing throughout the year
#define SouthElev 40.0
#define SouthAzi 180.0
//These are specific to the compass being used
//See maintenance document for determining offset
#define XOffset 16.88
#define YOffset -3.87
// Taken from example code
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculated here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
// This does change over time, last updated 3/15/2023
#define DECLINATION -11.57  // Declination (degrees) in Binghamton, NY.
*/
// Function Declarations
void Voltages();
void Currents();
//void TempAndHumid(); //Tracking
void Wind();
/* //Tracking
void RollElevation(float ax, float ay, float az); //Tracking
void Azimuth(float mx, float my);
void MoveSPAzi(float Azi);
void MoveSPElev(float Elev);
*/
void EnableMotor(int MotorNumber, int Direction);
void DisableMotor(int MotorNumber);
void ManualControl();
//void CheckLimitSwitches(); //Tracking
void TransferPiData();
void ReceivePiData(int suntime);


void setup() {
    TCCR0B = TCCR0B & (B11111000 | B00000010);
    Serial.begin(115200);  // Serial for printing output
    /* //Tracking
    Wire.begin();
    //Wire.setClock(56000);
    */
    Clock.begin();  // Activate RTC
    /*
     * NOTE ON RTC:
     * If the battery on the RTC dies and the RTC's date and time is off,
     * The date and time can be changed via the 'SetDateTime' example that comes
     * with the 'clock' library
     */

    // while (Serial.readStringUntil('\n') != "start")
    //     ;

    // Set Pin Modes
    //Pullup inputs are for switch controls
    pinMode(MANUAL, INPUT_PULLUP);
    pinMode(M2_EAST, INPUT_PULLUP);
    pinMode(M2_WEST, INPUT_PULLUP);
    pinMode(M1_UP, INPUT_PULLUP);
    pinMode(M1_DN, INPUT_PULLUP);
    pinMode(M1_PUL, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PUL, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    /* //Tracking
    pinMode(LIMIT_SIG_1, INPUT_PULLUP);
    pinMode(LIMIT_SIG_2, INPUT_PULLUP);
    */
    pinMode(InverterEnable, OUTPUT);
    /* //Tracking
    // Begin I2C Communication
    // Begin Communication with SHT30
    if (!sht30.begin(0x44)) {
        Serial.println("Couldn't find SHT30");
    }
    // Begin Communication with LSM9DS1 accelerometer
    if (!Comp.begin(LSM9DS1_A_C, LSM9DS1_C_C)) {
        Serial.println("Couldn't Find LSM9DS1 COMP");
    }
    if (!Accel.begin(LSM9DS1_A_A, LSM9DS1_C_A)) {
        Serial.println("Couldn't Find LSM9DS1 ACCEL");
    }
    */
    MyDateAndTime = Clock.read();
    // get initial times and position from pi
    ReceivePiData(1);
    //Turn ON inverter initially
    digitalWrite(InverterEnable, HIGH);
}

void loop() {
    // CHECK: Need delay?
    delay(16000);

    // Collect Data
    Voltages();
    Currents();
    Wind();
    /* //Tracking
    TempAndHumid();
    Comp.readMag();
    Azimuth(-Comp.my,-Comp.mx);
    Accel.readAccel();
    RollElevation(Accel.ax, Accel.ay, Accel.az);
    */

    // Determine Current Time
    MyDateAndTime = Clock.read();

    // Send Data to Pi
    // send data every loop ( 2 seconds )
    //if (MyDateAndTime.Second % 2 == 0){
    TransferPiData();
    //}

    if ((MyDateAndTime.Hour == 6) && (MyDateAndTime.Minute == 0)) {
        if (!morning_lockout) {
            // get sunrise/sunset times & positions for the day
            ReceivePiData(1);
            morning_lockout = 1;  // prevent repeated requests
        }
    }
    if ((MyDateAndTime.Hour == 6) && (MyDateAndTime.Minute == 1)) {
        morning_lockout = 0;  // clear lockout for the next day
    }

    if (MyDateAndTime.Minute % 30 == 0) {  // get new data every 30 mins
        if ((Clock.compareTimestamps(MyDateAndTime, Today_Sunrise) == 1) &&
            (Clock.compareTimestamps(MyDateAndTime, Today_Sunset) == -1)) {
            // ^ only get position data during the day
            ReceivePiData(0);
        }
    }

    //Attempt to prevent collision by checking proximity w limit switch
    //CheckLimitSwitches(); //CHECK vals //Tracking

    //Non-Tracking Behavior
    if (digitalRead(MANUAL) == HIGH){
        ManualControl();
        State = 0;
    }else if (BatteryTotalVoltage < 24.4) {
        digitalWrite(InverterEnable, LOW);
        State = 2;
    }else{
        digitalWrite(InverterEnable, HIGH);
        State = 7;
    }


    /* //Tracking -This is the behavior that we would've used if the tracking worked
    // Determine State
    if (digitalRead(MANUAL) == HIGH) {
        State = 0;
    //Must figure out what to do with limit switches in testing
    //Do we move the SP away when a limit switch is hit or just shut it all down?
    //CHECK
    } else if ((digitalRead(LIMIT_SIG_1) == LOW || NEAR_LIM1 == 1) || (digitalRead(LIMIT_SIG_2) == LOW || NEAR_LIM2 == 1)) {
        State = 1;
    } else if (BatteryTotalVoltage < 24.4) {
        State = 2;
    } else if (MyDateAndTime.Minute % 30 == 0) {
        digitalWrite(InverterEnable, HIGH); //CHECK see if it works
        AziSpace = 0;
        ElevSpace = 0;
        if (WindSpeed > 123) {
            State = 3;
        } else if (Temp < -5.0 && Humid > 70.0) { //These vals are educated guesses
            State = 4;
        } else if ((Today_Sunrise.Hour > MyDateAndTime.Hour) || (Today_Sunset.Hour < MyDateAndTime.Hour)) {
            State = 5;
        } else if (Humid > 70.0) {//These vals are educated guesses
            State = 6;
        } else {
            State = 7;
        }
    }

    // Perform State
    switch (State) {
        case 0:  // Manual Mode
            ManualControl();
        case 1:  // Limit Switch Trigger
        //If we want to have it more than just disable motors
        //(Which would have to be manually fixed)
        //We need to figure out where to move the motors and which switch is for which axis 
            if(digitalRead(LIMIT_SIG_1) == HIGH || NEAR_LIM1 == 1){
                DisableMotor(1);
                DisableMotor(2);
            }else{
                DisableMotor(1);
                DisableMotor(2);
            }
        case 2:  // Conserve Battery
            digitalWrite(InverterEnable, LOW);
            MoveSPElev(SouthElev);
            MoveSPAzi(SouthAzi);
        case 3:  // Protect from Wind
            MoveSPElev(WindElev);
            //Azi doesn't matter
        case 4:  // Protect from Snow
            MoveSPElev(SnowElev);
            MoveSPAzi(SnowAzi); //Azi matters only to avoid collision
        case 5:  // Set to Morning
            MoveSPElev(SunriseAzimuth);
            MoveSPAzi(SunriseElevation);
        case 6:  // Cloudy Weather
            MoveSPElev(SouthElev);
            MoveSPAzi(SouthAzi);
        case 7:  // Angle Towards Sun
            MoveSPElev(ElevationCommand);
            MoveSPAzi(AzimuthCommand);
    }
    */
}

void Voltages() {
    // Read the input on analog pin(s)
    // Convert analog readings (which range from 0-1023) to a voltage (0-55V)
    //CHECK
    PanelVoltage = analogRead(SP_VOLT) * (55 / 1024.0);             // max 55V
    BatteryTotalVoltage = analogRead(BATT24_VOLT) * (55 / 1024.0);  // max 55V
    BatteryOneVoltage = analogRead(BATT12_VOLT) * (55 / 1024.0);    // max 55V
}

void Currents() {
    // Get current measurements
    PanelCurrent = SP_CUR.mA_DC() / 1000.0;
    BatteryCurrent = BATT_CUR.mA_DC() / 1000.0;
    LoadCurrent = LOAD_CUR.mA_DC() / 1000.0;
}

/* //Tracking
void TempAndHumid() {
    Temp = sht30.readTemperature();
    Humid = sht30.readHumidity();
}
*/

void Wind() {
    // Taken from old code, must test
    //  Read the input on analog pin A15
    //CHECK
    float sensorValue15 = (analogRead(W_SIG) * (5.0 / 1023.0));
    if (sensorValue15 < 0.4) {
        WindSpeed = 0;
    } else {
        WindSpeed = (sensorValue15 - 0.4) * (32.4 / (2.0 - 0.4));
    }
}

/* //Tracking
// Calculate elevation (aka altitude/pitch) and roll
// Pitch/roll calculations take from this app note:
// https://web.archive.org/web/20190824101042/http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
void RollElevation(float ax, float ay, float az) {
    MeasuredRoll = atan2(ay, az);
    MeasuredElevation = atan2(-ax, sqrt(ay * ay + az * az));
    // Convert everything from radians to degrees:
    MeasuredElevation *= 180.0 / PI;
    MeasuredRoll *= 180.0 / PI;
    MeasuredElevation = -MeasuredElevation;
}

// Inputs should be negative
// Heading (aka yaw/azimuth) calculations taken from this app note:
// https://web.archive.org/web/20150513214706/http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void Azimuth(float mx, float my){
    //Subtract offset vals from measurements
    if (my-YOffset == 0)
        MeasuredAzimuth = (mx-XOffset < 0) ? PI : 0;
    else
        MeasuredAzimuth = atan2(mx - XOffset, my - YOffset);

    MeasuredAzimuth -= DECLINATION * PI / 180;

    if (MeasuredAzimuth > PI)
        MeasuredAzimuth -= (2 * PI);
    else if (MeasuredAzimuth < -PI)
        MeasuredAzimuth += (2 * PI);
    
    // Convert everything from radians to degrees:
    MeasuredAzimuth *= 180.0 / PI;
    if(MeasuredAzimuth < 0){
        MeasuredAzimuth = MeasuredAzimuth + 360;
    }
}

void MoveSPElev(float Elev) {
    // If less or greater than the range, move the motors to fix
    if (Elev - ElevRange - ElevSpace >= MeasuredElevation) {
        EnableMotor(
            2, 2);  // Need to determine which direction leads to which angle
                    // change CHECK: if second variable should be 1 or 2
    } else if (Elev + ElevRange + ElevSpace <= MeasuredElevation) {
        EnableMotor(2, 1);
    } else {
        DisableMotor(2);
        ElevSpace = 5;
    }
}

void MoveSPAzi(float Azi) {
    // If less or greater than the range, move the motors to fix
    if (Azi - AziRange - AziSpace >= MeasuredAzimuth) {
        EnableMotor(1, 1);  // Need to determine which direction leads to which
                            // angle change
    } else if (Azi + AziRange + AziSpace <= MeasuredAzimuth) {
        EnableMotor(1, 2);
    } else {
        DisableMotor(1);
        AziSpace = 5;
    }
}
*/

/*
 * Motor functions taken from 2018 code
 */
// Enable Motor Rotation
void EnableMotor(int MotorNumber, int Direction) {
    if (MotorNumber == 1 && M1Running == 0) {
        if (Direction == 1) {
            digitalWrite(M1_DIR, LOW);
        } else {
            digitalWrite(M1_DIR, HIGH);
        }
        delayMicroseconds(100);
        analogWrite(M1_PUL, 127);
        M1Running = 1;
    } else if (MotorNumber == 2 && M2Running == 0) {
        if (Direction == 1) {
            digitalWrite(M2_DIR, LOW);
        } else {
            digitalWrite(M2_DIR, HIGH);
        }
        delayMicroseconds(100);
        analogWrite(M2_PUL, 127);
        M2Running = 1;
    } else {
        M1Running = M1Running;
        M2Running = M2Running;
    }
}
// Disable Motor Rotation
void DisableMotor(int MotorNumber) {
    if (MotorNumber == 1 && M1Running == 1) {
        digitalWrite(M1_PUL, LOW);
        M1Running = 0;
    } else if (MotorNumber == 2 && M2Running == 1) {
        digitalWrite(M2_PUL, LOW);
        M2Running = 0;
    } else {
        M1Running = M1Running;
        M2Running = M2Running;
    }
}

void ManualControl() {
    if (digitalRead(M1_UP) == LOW) {
        EnableMotor(2, 1);
    } else if (digitalRead(M1_DN) == LOW) {
        EnableMotor(2, 2);
    }
    if (digitalRead(M2_EAST) == LOW) {
        EnableMotor(1, 1);
    } else if (digitalRead(M2_WEST) == LOW) {
        EnableMotor(1, 2);
    }
}
/* //Tracking
void CheckLimitSwitches() {
    // See if near limit switch to avoid triggering it, determine w measured vals
    if (MeasuredElevation > LimitElev) {  // CHECK: Greater or less than?
        NEAR_LIM1 = 1;
    } else {
        NEAR_LIM1 = 0;
    }
    if (MeasuredAzimuth > LimitAzi) {  // CHECK: Greater or less than?
        NEAR_LIM2 = 1;
    } else {
        NEAR_LIM2 = 0;
    }
}
*/

void TransferPiData() {
    char system_status[25];
    if (State == 1) {
        sprintf(system_status, "Maintenance");
    } else if (State == 2) {
        sprintf(system_status, "High Wind");
    } else if (State == 3) {
        sprintf(system_status, "Low Battery");
    } else if (State == 4) {
        sprintf(system_status, "Cold Weather");
    } else if (State == 5) {
        sprintf(system_status, "Overnight");
    } else {
        sprintf(system_status, "Normal");
    }

    char AzimMode[25];
    if (State == 0) {
        sprintf(AzimMode, "Automatic");
    } else {
        sprintf(AzimMode, "Manual");
    }

    char AzimStatus[25];
    if (M1Running == 1) {
        if (digitalRead(M1_DIR)) {
            sprintf(AzimStatus, "CCW");
        } else {
            sprintf(AzimStatus, "CW");
        }
    } else {
        sprintf(AzimStatus, "OFF");
    }

    char ElevMode[25];
    if (State == 0) {
        sprintf(ElevMode, "Automatic");
    } else {
        sprintf(ElevMode, "Manual");
    }
    char ElevStatus[25];
    if (M2Running == 1) {
        if (digitalRead(M2_DIR)) {
            sprintf(ElevStatus, "CCW");
        } else {
            sprintf(ElevStatus, "CW");
        }
    } else {
        sprintf(ElevStatus, "OFF");
    }
    char date_time[64];
    DateTime current_time;
    current_time = Clock.read();
    sprintf(date_time, "20%02i_%02i_%02i_%02i_%02i_%02i", current_time.Year,
            current_time.Month, current_time.Day, current_time.Hour,
            current_time.Minute, current_time.Second);

            // single bit difference btwn bcd 33 and bcd 23 @ bit 4
            // 2 bits diff btwn bcd 03 and bcd 33 @ bits 4 and 5

    // sprintf(date_time, "2022-02-20_02:55:32");
    Serial.println("LOG");
    char buffer[1024];
    sprintf(
        buffer,
        "{'Date_Time': '%s', 'System_Status': '%s', 'Solar_Panel_Voltage': '%f', "
        "'Solar_Panel_Current': '%f', 'Solar_Panel_Power': '%f', "
        "'Battery_One_Voltage': '%f', 'Battery_Two_Voltage': '%f', "
        "'Battery_Total_Voltage': '%f', 'Battery_Total_Power' : '%f', "
        "'Load_Voltage': '%f', 'Load_Current': '%f', 'Load_Power': '%f', "
        "'Windspeed': '%f', 'Outdoor_Temp': '%f', "
        "'Outdoor_Humidity': '%f', 'Azimuth_Reading': '%f', 'Azimuth_Command': '%i', "
        "'Azimuth_Motor_Mode': '%s', 'Azimuth_Motor_Status': '%s', "
        "'Elevation_Reading': '%f', 'Elevation_Command': '%i', "
        "'Elevation_Motor_Mode': '%s', 'Elevation_Motor_Status': '%s'}",
        date_time, system_status, PanelVoltage, PanelCurrent,
        PanelVoltage * PanelCurrent, BatteryOneVoltage,
        BatteryTotalVoltage - BatteryOneVoltage, BatteryTotalVoltage,
        BatteryTotalVoltage * BatteryCurrent, BatteryTotalVoltage, LoadCurrent,
        BatteryTotalVoltage * LoadCurrent, WindSpeed, Temp, Humid,
        MeasuredAzimuth, AzimuthCommand, AzimMode, AzimStatus,
        MeasuredElevation, ElevationCommand, ElevMode, ElevStatus);
    Serial.println(buffer);
}

void ReceivePiData(int suntime) {
    int AziBuffer;
    int ElevBuffer;
    Serial.println("new_position");
    AziBuffer = Serial.readStringUntil('\n').toInt();
    ElevBuffer = Serial.readStringUntil('\n').toInt();
    AzimuthCommand = AziBuffer;
    ElevationCommand = ElevBuffer;
    if (suntime == 1) {  // CHANGE
        Serial.println("new_times");
        String today_sunrise;
        String today_sunset;
        int sunrise_azimuth;
        int sunrise_elevation;
        today_sunrise = Serial.readStringUntil('\n');
        today_sunset = Serial.readStringUntil('\n');
        sunrise_azimuth = Serial.readStringUntil('\n').toInt();
        sunrise_elevation = Serial.readStringUntil('\n').toInt();
        today_sunrise.trim();
        today_sunset.trim();
        SunriseAzimuth = sunrise_azimuth;
        SunriseElevation = sunrise_elevation;
        //  actually receive data from raspi, sunrise then sunset in HH:MM:SS
        Today_Sunrise = MyDateAndTime;
        Today_Sunrise.Hour = today_sunrise.substring(6, 7).toInt();
        Today_Sunrise.Minute = today_sunrise.substring(3, 4).toInt();
        Today_Sunrise.Second = today_sunrise.substring(0, 1).toInt();

        Today_Sunset = MyDateAndTime;
        Today_Sunset.Hour = today_sunset.substring(6, 7).toInt();
        Today_Sunset.Minute = today_sunset.substring(3, 4).toInt();
        Today_Sunset.Second = today_sunset.substring(0, 1).toInt();
    }
}
