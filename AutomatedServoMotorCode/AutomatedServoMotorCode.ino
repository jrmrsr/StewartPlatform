/* =================================================================================================== 
 *  Several lines of code are accredited to the Adafruit PWM Servo Driver Library example code.
 *  -> Add Adafruit PWM shield library
 *  
 *  Adapted by Jose Rondon and group 6
 =================================================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal.h>
// TODO FIND ENCONDER VALUES FOR TOWERPRO MG995
// Motor Limits
// Pulse length counts are out of 4096
// Indexed from servo 1 - 6
const int SERVOMINS[6] = {160, 165, 105, 130, 135, 155};
const int SERVOMAXS[6] = {500, 540, 370, 430, 470, 500};
const int SERVOCHG = 5;
const int SERVO_INTERVAL_WORDS = 2500;
const int SERVO_INTERVAL_NUMBERS = 5000;

// Gate Reading Limit
const int GATE_LIMIT = 100;

// Photocell Variables
// Pins
const int GATE_PIN_1 = A10;
const int GATE_PIN_2 = A11;
const int GATE_PIN_3 = A12;
const int GATE_PIN_4 = A13;
const int GATE_PIN_5 = A14;
const int GATE_PIN_6 = A15;
const int JOYSTICK_1_1 = A8; // slider variable connecetd to analog pin 0
const int JOYSTICK_1_2 = A9; // slider variable connecetd to analog pin 1

// Base Parameters
const int LINKAGE_LENGTH = 90;
const int SERVO_ARM_LENGTH = 35;
const float BASE_POSITIONS[6][3] = {
    {83.5, 32.81, .0},
    {-13.3, 88.72, 0.0},
    {-70.17, 55.91, 0.0},
    {-70.17, -55.91, 0.0},
    {-13.33, -88.71, 0.0},
    {83.5, -32.81, 0.0}};
const float PLATFORM_POSITIONS[6][3] = {
    {42.7, 61.95, 0.0},
    {32.3, 67.95, 0.0},
    {-75.0, 6.0, 0.0},
    {-75.0, -6.0, 0.0},
    {32.3, -67.95, 0.0},
    {42.7, -61.95, 0.0}};

const float BETA[3] = {0.0, 120.0, 240.0};
const int MATRIX_ROWS = 3;

// Angles
float theta = 0.0;
float phi = 0.0;
float psi = 0.0;

// Home height Vector (Also zt)
float home_height[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Runs (xp-xb)^2 and (yp-yb)^2
float base_platform_deltas[6][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

// Reading
int gate_reading_1 = 0;
int gate_reading_2 = 0;
int gate_reading_3 = 0;
int gate_reading_4 = 0;
int gate_reading_5 = 0;
int gate_reading_6 = 0;
int joystick_reading_1_1 = 0;
int joystick_reading_1_2 = 0;
bool joystick_on = false;
bool keyboard_off = false;
// Initializing PWM Shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Initializing lCD
LiquidCrystal lcd(19, 18, 47, 49, 51, 53); /// REGISTER SELECT PIN,ENABLE PIN,D4 PIN,D5 PIN, D6 PIN, D7 PIN
// Message Timing
unsigned long msg_servo_1 = 0;
unsigned long msg_servo_2 = 0;
/*
NOTE
TO CONVERT DEGREES TO PULSE LENGTH, USE THE FOLLOWING
pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
from https://learn.adafruit.com/16-channel-pwm-servo-driver/using-the-adafruit-library 
*/
int servo_settings[6] = {0, 0, 0, 0, 0, 0}; // PWM var
String input = "0";

void setup()
{
    Serial.begin(9600);    // opens serial port, sets data rate to 9600 bps
    Serial.setTimeout(10); // change default (1000ms) to have faster response time
    pwm.begin();
    // TRY TO USE 50 HZ, LOWEST IS 40 HZ
    pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
    lcd.begin(16, 2);
    int mid_1 = 0;
    int mid_2 = 0;
    for (int i = 0; i < 6; i++)
    {
        // Changing the order of the servo settings array allows us to change the orientation of the maze if
        // we use delta angle changes for our servos
        Serial.println(SERVOMINS[i]);
        Serial.println(SERVOMAXS[i]);
        mid_1 = (SERVOMINS[i] + SERVOMAXS[i]);
        mid_2 = mid_1 / 2;
        servo_settings[i] = mid_2;
    }
    for (int i = 0; i < 6; i++)
    {
        base_platform_deltas[i][0] = pow((BASE_POSITIONS[i][0] * PLATFORM_POSITIONS[i][0]), 2); // (xp-xb)^2
        base_platform_deltas[i][1] = pow((BASE_POSITIONS[i][1] * PLATFORM_POSITIONS[i][1]), 2); // (xp-xb)^2
    }
    ServoValues();
    SetServos(servo_settings[0], servo_settings[1], servo_settings[2], servo_settings[3], servo_settings[4], servo_settings[5]);
    // SetServos(350, 350, 350, 350, 350, 350);
    Serial.println("Would you like me to solve this maze for you? (y/n/c)");
}

void loop()
{
    keyboard_off = JoysticksOn();
    // Check keyboard string
    // ServoValues();
    if (Serial.available() > 0 && !keyboard_off) // Joystick input is not detected)
    {
        input = Serial.readString();
        Serial.println(input);

        // Add in pause keystroke after every servo change once the marble passes
        // a photocell gate for debugging
        switch (input[0])
        {
        case 'y':
            SolveMaze();
            break;
        case 'c':
            SetServos(90, 90, 90, 90, 90, 90);
            ReadAllPhotocells();
            break;
        case 'n':
            Serial.println("Okay I will just wait until you say yes.");
            break;
        case 's':
            SetServos(350, 350, 350, 350, 350, 350);
            break;
        case '-':
            SetServos(SERVOMINS[0], SERVOMINS[1], SERVOMINS[2], SERVOMINS[3], SERVOMINS[4], SERVOMINS[5]);
            break;
        case '+':
            SetServos(SERVOMAXS[0], SERVOMAXS[1], SERVOMAXS[2], SERVOMAXS[3], SERVOMAXS[4], SERVOMAXS[5]);
            break;
        case 'p':
            ReadAllPhotocells();
            break;
        }
    }
    else
    {
        // Manual control
        ManualControl();
    }
    if (millis() > msg_servo_1 + SERVO_INTERVAL_WORDS)
    {
        msg_servo_1 = millis();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Servo Values:");
    }
    if (millis() > msg_servo_2 + SERVO_INTERVAL_NUMBERS)
    {
        msg_servo_2 = millis();
        ServoValues();
    }
    input = "";
    delay(40); // servos cannot receive pwm changes any quicker than this
}

void ServoValues()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("[");
    for (int i = 0; i < 3; i++)
    {
        lcd.print(servo_settings[i]);
        lcd.print(",");
    }
    lcd.setCursor(0, 1);
    for (int i = 3; i < 5; i++)
    {
        lcd.print(servo_settings[i]);
        lcd.print(",");
    }
    lcd.print(servo_settings[5]);
    lcd.print("]");
}

// Takes in motor angle in degrees and then returns an array of PWM values for motors
// ADD A DEAD BAND DELAY TO REDUCE JITTER
void SetServos(int motor_1, int motor_2, int motor_3, int motor_4, int motor_5, int motor_6)
{
    // temp array
    int temp_servo_settings[6] = {motor_1,
                                  motor_2,
                                  motor_3,
                                  motor_4,
                                  motor_5,
                                  motor_6};

    for (int i = 0; i < 6; i++)
    {
        // Check that temp_servo_settings is not NULL and it is within our angle bounds
        // *NOTE* three checks are added for testing purposes. If they take too long to compute, they will be removed.
        if (temp_servo_settings[i] != SERVOMINS[i] && temp_servo_settings[i] >= 0 && temp_servo_settings[i] < 180)
        {
            // ADD SERVO DEAD BAND (NO CHANGE FOR SMALL DELTAS)
            servo_settings[i] = map(temp_servo_settings[i], 0, 179, SERVOMINS[i], SERVOMAXS[i]);
        }
        else
        {
            servo_settings[i] = temp_servo_settings[i];
        }
        pwm.setPWM(i + 1, 0, servo_settings[i]); // added +1 to match PWM port numbering (pins 1..6 used)
    }
}

void SolveMaze()
{
    // First gate
    ServoValues(); // Servo values are used for debugging
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to first gate");
    int gate_reading = analogRead(GATE_PIN_1); // Initialize gate reading for while loop
    // Maintain servos at required position using the while loop as a blocker
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_1);
    }

    // Second gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the second gate");
    gate_reading = analogRead(GATE_PIN_2);
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_2);
    }

    // Third gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the third gate");
    gate_reading = analogRead(GATE_PIN_3);
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_3);
    }

    // Fourth gate
    ServoValues();
    SetServos(100, 100, 100, 500, 100, 100);
    Serial.println("Heading to the fourth gate");
    gate_reading = analogRead(GATE_PIN_4);
    while (gate_reading > GATE_LIMIT)
    {
        ReadAllPhotocells();
        gate_reading = analogRead(GATE_PIN_4);
    }
}

void ReadAllPhotocells()
{
    gate_reading_1 = analogRead(GATE_PIN_1);
    gate_reading_2 = analogRead(GATE_PIN_2);
    gate_reading_3 = analogRead(GATE_PIN_3);
    gate_reading_4 = analogRead(GATE_PIN_4);
    gate_reading_5 = analogRead(GATE_PIN_5);
    gate_reading_6 = analogRead(GATE_PIN_6);
    Serial.print("Photocell Values 1-6: ");
    Serial.print(gate_reading_1);
    Serial.print(", ");
    Serial.print(gate_reading_2);
    Serial.print(", ");
    Serial.print(gate_reading_3);
    Serial.print(", ");
    Serial.print(gate_reading_4);
    Serial.print(", ");
    Serial.print(gate_reading_5);
    Serial.print(", ");
    Serial.println(gate_reading_6);
}

void ManualControl()
{
    // index is w * h
    static double rotation_matrix[9] = {0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ReadJoysticks();
    theta = DegToRad(theta);
    phi = DegToRad(phi);
    psi = DegToRad(psi);
    CalcRotation(rotation_matrix, psi, theta, phi);
    for (int i = 0; i < 6; i++)
    {
        home_height[i] = sqrt((LINKAGE_LENGTH * LINKAGE_LENGTH + SERVO_ARM_LENGTH * SERVO_ARM_LENGTH - base_platform_deltas[i][0] - base_platform_deltas[i][1] - PLATFORM_POSITIONS[i][2]));
    }
}

void ReadJoysticks()
{
    joystick_reading_1_1 = analogRead(JOYSTICK_1_1);
    delay(100);
    joystick_reading_1_2 = analogRead(JOYSTICK_1_2);
    Serial.print("Joystick Values 1-6: ");
    Serial.print(joystick_reading_1_1);
    Serial.print(",");
    Serial.println(joystick_reading_1_2);
}

bool JoysticksOn()
{
    if (joystick_reading_1_1 || joystick_reading_1_2)
    {
        joystick_on = true;
    }
    else
    {
        joystick_on = false;
    }
    return joystick_on;
}

// ServoAngles(&alphas, ) <- pass in variables as such
void ServoAngles(float *alphas, int alpha_size, float *base_coord, float *platform_coord, float Beta)
{
    // static float li =
}

float DegToRad(float deg)
{
    return (deg * 71) / 4068;
}

// Make sure angles are in radians
void CalcRotation(double rot[], float psi, float theta, float phi)
{
    // index as width * row + col
    rot[(3*0+0)] = cos(psi) * cos(theta);
    rot[(3*0+1)] = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
    rot[(3+0+2)] = sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi);
    rot[(3*1+0)] = sin(psi) * cos(theta);
    rot[(3*1+1)] = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
    rot[(3*1+2)] = -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi);
    rot[(3*2+0)] = -sin(theta);
    rot[(3*2+1)] = cos(theta) * sin(phi);
    rot[(3*2+2)] = cos(theta) * cos(phi);
}

// What is q or p?
void MatrixMultRotation(float *rot, float *platform, float *result)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i] += rot[(3*i+j)] * platform[j];
        }
    }
}

void MatrixSummation(float M1[], float M2[], int rows, int col, float results[])
{
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < col; j++)
        {
            results[((col+1)*i+j)] += M1[((col+1)*i+j)] + M2[((col+1)*i+j)];
        }
    }
}