// Libraries
#include <ACS712.h>  // Library for current sensors
#include <Adafruit_SHT31.h>  // Library for Temp and Humidity Sensor(SHT30 is compatible)
#include <Arduino.h>
#include <DS3231_Simple.h>    // Library for Real Time Clock
#include <SPI.h>              // Library for communication (may be unused)
#include <SparkFunLSM9DS1.h>  // Library for accelerometer and compass
#include <Wire.h>             // Library for I2C communication

/*
 * FIX ALL PIN LOCATIONS
 * CHECK MOTOR1/MOTOR2/ELEV/AZI
 */

// Non-Pin Setup for Components
// Pure Inputs
// Temperature/Humidity Sensor
Adafruit_SHT31 sht30 = Adafruit_SHT31();
// Compass/accelerometer
//  Taken from old code
LSM9DS1 accelComp;
// I2C addresses for compass and accelerometer communication
#define LSM9DS1_C 0x1E  // compass
#define LSM9DS1_A 0x6B  // accelerometer
// Taken from example code
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculated here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -12.513  // Declination (degrees) in Binghamton, NY.
// Pure Outputs
// Input/Outputs
// Real Time Clock
DS3231_Simple Clock;
DateTime MyDateAndTime;
// Pins
// Analog
int SP_CUR = A0;       // Solar Panel Current
int BATT_CUR = A1;     // Battery Current
int LOAD_CUR = A2;     // Load Current
int SP_VOLT = A3;      // Voltage of Solar Panel
int BATT24_VOLT = A4;  // Voltage of both batteries
int BATT12_VOLT = A5;  // Voltage of 1st batteries
// ACS712 PanelSensor(ACS712_20A, A1); //This is how the previous code did it,
// but we got it working without insert other current sensor inputs here
int W_SIG = A15;  // Wind Meter (Anenometer)
// Digital
// Manual Controls for both Motors
int MANUAL = 2;  // Activates Motor Manual mode
int M2_EAST = 3;
int M2_WEST = 4;
int M1_UP = 5;
int M1_DN = 6;
// Motor Driver Inputs
int M1_PUL = 40;
int M1_DIR = 41;
int M2_PUL = 42;
int M2_DIR = 43;
int LIMIT_SIG_1 = 46;  // Limit Switch 1(CHECK if elev or Azi)
int LIMIT_SIG_2 = 47;  // Limit Switch 2(CHECK if elev or Azi)
// Other
int InverterDisable = 30;
// Global Variables
// Sensors
double Temp = 0;
double Humid = 0;
double PanelCurrent = 0;
double BatteryCurrent = 0;
double LoadCurrent = 0;
double BatteryTotalVoltage = 0;
double PanelVoltage = 0;
double BatteryOneVoltage = 0;
double WindSpeed = 0;
double MeasuredAzimuth = 0.0;    // AKA Yaw or Heading
double MeasuredElevation = 0.0;  // AKA Zenith or Pitch
double MeasuredRoll = 0.0;       // Unused, solar panels don't roll
// Motors
int M1Running = 0;
int M2Running = 0;
// Finite State Machine
int State = 0;
int WindyWeather = 0;
int SnowyWeather = 0;
int CloudyWeather = 0;
int Night = 0;
int Completed = 0;
// Data Transfer
int PiComm = 0;
// Definitions
// ADD: Must determine proper defintion values experimentally
#define ElevRange 5.0
#define AziRange 5.0
#define SnowElev 0.0
#define SnowAzi 0.0
#define LimitElev 0.0
#define LimitAzi 0.0
#define WindElev 0.0
#define WindAzi 0.0
#define SouthElev 0.0
#define SouthAzi 0.0

// Function Declarations
void Voltages();
void Currents();
void TempAndHumid();
void Wind();
void Attitude(float ax, float ay, float az, float mx, float my, float mz);
void MoveSPAzi(float Azi);
void MoveSPElev(float Elev);
void EnableMotor(int MotorNumber, int Direction);
void DisableMotor(int MotorNumber);
void ManualControl();
void CheckLimitSwitches();
void CheckLimitElev();
void TransferPiData();
void ReceivePiData();

void setup() {
    Serial.begin(9600);  // Serial for printing output
    Clock.begin();       // Activate RTC
    /*
     * NOTE ON RTC:
     * If the battery on the RTC dies and the RTC's date and time is off,
     * The date and time can be changed via the 'SetDateTime' example that comes
     * with the 'clock' library
     */
    // Set Pin Modes
    pinMode(M2_EAST, INPUT);
    pinMode(M2_WEST, INPUT);
    pinMode(MANUAL, INPUT);
    pinMode(M1_UP, INPUT);
    pinMode(M1_DN, INPUT);
    pinMode(M1_PUL, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PUL, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(LIMIT_SIG_1, INPUT);
    pinMode(LIMIT_SIG_2, INPUT);
    pinMode(InverterDisable, OUTPUT);
    // Begin I2C Communication
    // Begin Communication with SHT30
    if (!sht30.begin(0x44)) {
        Serial.println("Couldn't find SHT30");
    }
    // Begin Communication with LSM9DS1 accelerometer
    if (!accelComp.begin())  // with no arguments, this uses default addresses
                             // (AG:0x6B, M:0x1E) and i2c port (Wire).
    {
        Serial.println("Couldn't Find LSM9DS1.");
    }
}

void loop() {
    // Collect Data
    Voltages();
    Currents();
    TempAndHumid();
    Wind();
    Attitude(accelComp.ax, accelComp.ay, accelComp.az, accelComp.mx,
             accelComp.my, accelComp.mz);

    // Send Data to Pi
    MyDateAndTime = Clock.read();
    if (MyDateAndTime.Minute % 2 ==
        0) {  // Transfer data every 2min CHECK: if this is the desired interval
        if (PiComm ==
            0) {  // Prevents data being transferred multiple times in a row
            TransferPiData();
            ReceivePiData();
            PiComm = 1;
        }
    } else {
        PiComm = 0;
    }

    // Determine State
    // CHECK: Is another State for subzero temps needed?
    // Could just put the actions done in switch() in here
    // ADD: May want to have the State 'persist' for a little bit, to prevent
    // the solar panel from switching States too often ADD: Maybe also have the
    // State only be triggered after multiple checks, to avoid accidently
    // putting it in a wrong mode

    if (digitalRead(MANUAL) == HIGH) {
        State = 0;
    } else if (Completed == 1 || (LIMIT_SIG_1 == 1 || LIMIT_SIG_2 == 1)) {
        State = 1;
    } else if (BatteryTotalVoltage <
               20) {  // Voltage level is an estimate of charge, must determine
                      // the voltage to determine the charge. Current val is a
                      // placeholder
        State = 2;
    } else if (MyDateAndTime.Minute % 30 == 0) {
        digitalWrite(InverterDisable, LOW);
        if (WindyWeather == 1) {
            State = 3;
        } else if (SnowyWeather == 1) {
            State = 4;
        } else if (Night == 1) {
            State = 5;
        } else if (CloudyWeather == 1) {
            State = 6;
        } else {
            State = 7;
            Completed = 0;
        }
    }

    // Check if near limit switches
    CheckLimitSwitches();

    // Perform State
    switch (State) {
        case 0:  // Manual Mode
            ManualControl();
        case 1:  // Disable Motors
            DisableMotor(1);
            DisableMotor(2);
        case 2:  // Conserve Battery
            // Move SP to southern tilt and send signal to shut off inverter
            digitalWrite(InverterDisable, HIGH);
            MoveSPElev(SouthElev);
            MoveSPAzi(SouthAzi);
        case 3:  // Protect from Wind
            // Insert set values for the most horizontal SP position
            MoveSPElev(WindElev);
            MoveSPAzi(WindAzi);
        case 4:  // Protect from Snow
            // Insert set values for the most vertical SP position
            MoveSPElev(SnowElev);
            MoveSPAzi(SnowAzi);
            // Probably shouldn't trigger this the moment it snows- flurries are
            // common in bing
        case 5:  // Set to Morning
                 // Use values from Pi for next morning's sunrise
                 // MoveSPElev(float Elev);
                 // MoveSPAzi(float Azi);
        case 6:  // Cloudy Weather
            // Move SP to southern tilt, do NOT shut off inverter
            MoveSPElev(SouthElev);
            MoveSPAzi(SouthAzi);
        case 7:  // Angle Towards Sun
            int a = 0;
            // Use values taken from Pi
            // MoveSPElev(float Elev);
            // MoveSPAzi(float Azi);
    }
}

void Voltages() {
    // Read the input on analog pin(s)
    // Convert the analog readings (which range from 0 - 1023) to a voltage (0 -
    // 5V)
    PanelVoltage = analogRead(SP_VOLT) * (55 / 1023);             // max 55V
    BatteryTotalVoltage = analogRead(BATT24_VOLT) * (55 / 1023);  // max 55V
    BatteryOneVoltage = analogRead(BATT12_VOLT) * (55 / 1023);    // max 55V
}

void Currents() {
    // Get running average of current measurements
    // PanelCurrent = (PanelCurrent + PanelSensor.getCurrentDC()) / 2; //This is
    // how the previous code did it, but we got it working without
    PanelCurrent = analogRead(SP_CUR) * (5 / 1023);
    BatteryCurrent = analogRead(BATT_CUR) * (5 / 1023);
    LoadCurrent = analogRead(LOAD_CUR) * (5 / 1023);
}

void TempAndHumid() {
    Temp = sht30.readTemperature();
    Humid = sht30.readHumidity();
}

void Wind() {
    // Taken from old code, must test
    //  Read the input on analog pin A15
    float sensorValue15 = (analogRead(W_SIG) * (5.0 / 1023.0));
    if (sensorValue15 < 0.4) {
        WindSpeed = 0;
    } else {
        WindSpeed = (sensorValue15 - 0.4) * (32.4 / (2.0 - 0.4));
    }
}
/*
  Attitude() is adapted from LSM9DS1_Basic_I2C Example
  We should be able to just use one accelerometer's pitch/roll/heading to
  determine the position of the solar panel. The proper pitch/roll/heading can
  only be determined once the accelerometer is in position on the solar panel
  Could add a check for the pitch/roll/heading right before hitting the limit
  switch, as an additional failsafe
*/
// Calculate elevation(aka altitude/pitch), roll, and azimuth(aka yaw/heading)
// of the solar panel Pitch/roll calculations take from this app note:
// https://web.archive.org/web/20190824101042/http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
// Heading calculations taken from this app note:
// https://web.archive.org/web/20150513214706/http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void Attitude(float ax, float ay, float az, float mx, float my, float mz) {
    MeasuredRoll = atan2(ay, az);
    MeasuredElevation = atan2(-ax, sqrt(ay * ay + az * az));

    MeasuredAzimuth;
    if (my == 0)
        MeasuredAzimuth = (mx < 0) ? PI : 0;
    else
        MeasuredAzimuth = atan2(mx, my);

    MeasuredAzimuth -= DECLINATION * PI / 180;

    if (MeasuredAzimuth > PI)
        MeasuredAzimuth -= (2 * PI);
    else if (MeasuredAzimuth < -PI)
        MeasuredAzimuth += (2 * PI);

    // Convert everything from radians to degrees:
    MeasuredAzimuth *= 180.0 / PI;
    MeasuredElevation *= 180.0 / PI;
    MeasuredRoll *= 180.0 / PI;
}

void MoveSPElev(float Elev) {
    // If less or greater than the range, move the motors to fix
    if (Elev - ElevRange >= MeasuredElevation) {
        EnableMotor(
            2, 1);  // Need to determine which direction leads to which angle
                    // change CHECK: if second variable should be 1 or 2
    } else if (Elev + ElevRange >= MeasuredElevation) {
        EnableMotor(2, 2);
    } else {
        DisableMotor(2);
    }
}

void MoveSPAzi(float Azi) {
    // If less or greater than the range, move the motors to fix
    if (Azi - AziRange >= MeasuredAzimuth) {
        EnableMotor(1, 1);  // Need to determine which direction leads to which
                            // angle change
    } else if (Azi + AziRange <= MeasuredAzimuth) {
        EnableMotor(1, 2);
    } else {
        DisableMotor(1);
    }
}

/*
 * Motor functions taken from 2018 code
 */
// Enable Motor Rotation
void EnableMotor(int MotorNumber, int Direction) {
    if (MotorNumber == 1 && M1Running == 0) {  // CHECK:CCW or CW?
        if (Direction == 1) {
            digitalWrite(M1_DIR, LOW);
        } else {
            digitalWrite(M1_DIR, HIGH);
        }
        delayMicroseconds(100);
        analogWrite(M1_PUL, 127);
        M1Running = 1;
    } else if (MotorNumber == 2 && M2Running == 0) {  // CHECK: CCW or CW?
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
    if (digitalRead(M1_UP) == HIGH) {
        EnableMotor(2, 1);
    } else if (digitalRead(M1_DN) == HIGH) {
        EnableMotor(2, 2);
    }
    if (digitalRead(M2_EAST) == HIGH) {
        EnableMotor(1, 1);
    } else if (digitalRead(M2_WEST) == HIGH) {
        EnableMotor(1, 2);
    }
}

void CheckLimitSwitches() {
    // See if near limit switch to avoid triggering it, figure out via measured
    // vals
    if (MeasuredElevation > LimitElev) {  // CHECK: Greater or less than?
        LIMIT_SIG_1 = 1;
    } else {
        LIMIT_SIG_1 = 0;
    }
    if (MeasuredAzimuth > LimitAzi) {  // CHECK: Greater or less than?
        LIMIT_SIG_2 = 1;
    } else {
        LIMIT_SIG_2 = 0;
    }
}

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
    char date_time[32];
    DateTime current_time;
    current_time = Clock.read();
    sprintf(date_time, "20%i-%i-%i_%i:%i:%i", current_time.Year,
            current_time.Month, current_time.Day, current_time.Hour,
            current_time.Minute, current_time.Second);
    // sprintf(date_time, "2022-02-20_02:55:32");
    Serial.println("LOG");
    char buffer[1024];
    sprintf(
        buffer,
        "{'Date_Time': %s, 'System_Status': %s, 'Solar_Panel_Voltage': %f, "
        "'Solar_Panel_Current': %f, 'Solar_Panel_Power': %f, "
        "'Battery_One_Voltage': %f, 'Battery_Two_Voltage': %f, "
        "'Battery_Total_Voltage': %f, 'Battery_Total_Power' : %f, "
        "'Load_Voltage': %f, 'Load_Current': %f, 'Load_Power': %f, "
        "'Windspeed': %f, 'Outdoor_Temp': %f, "
        "'Outdoor_Humidity': %f, 'Azimuth_Reading': %f, 'Azimuth_Command': %f, "
        "'Azimuth_Motor_Mode': %s, 'Azimuth_Motor_Status': %s, "
        "'Elevation_Reading': %f, 'Elevation_Command': %f, "
        "'Elevation_Motor_Mode': %s, 'Elevation_Motor_Status': %s}",
        date_time, system_status, PanelVoltage, PanelCurrent,
        PanelVoltage * PanelCurrent, BatteryOneVoltage,
        BatteryTotalVoltage - BatteryOneVoltage, BatteryTotalVoltage,
        BatteryTotalVoltage * BatteryCurrent, BatteryTotalVoltage, LoadCurrent,
        BatteryTotalVoltage * LoadCurrent, BatteryTotalVoltage, LoadCurrent,
        BatteryTotalVoltage * LoadCurrent, WindSpeed, Temp, Humid,
        MeasuredAzimuth, AzimuthCommand, AzimMode, AzimStatus,
        MeasuredElevation, ElevationCommand, ElevMode, ElevStatus);
    Serial.println(buffer);
}

void ReceivePiData() {
    // ADD: Could try to get weather data to predict rain,snow, cloudy weather,
    // azimuth, elevation
}
