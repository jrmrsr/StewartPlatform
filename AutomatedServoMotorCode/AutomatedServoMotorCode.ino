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
// const int SERVOMINS[6] = {150, 165, 160, 130, 135, 155};
// const int SERVOMAXS[6] = {500, 540, 500, 500, 470, 500};

// Test
const int SERVOMINS[6] = {150, 150, 150, 150, 150, 150};
const int SERVOMAXS[6] = {500, 500, 500, 500, 500, 500};
const double SERVOCHG = 1.0;
const int LCD_CHANGE_DELAY = 3000;

// Gate Reading Limit
const int GATE_LIMIT = 100;

// Photocell Variables
// Pins
const int GATE_PIN_1 = A0;
const int GATE_PIN_2 = A1;
const int GATE_PIN_3 = A12;
const int GATE_PIN_4 = A13;
const int GATE_PIN_5 = A14;
const int GATE_PIN_6 = A15;
const int JOYSTICK_1_1 = A8;      // slider variable connecetd to analog pin A8
const int JOYSTICK_1_2 = A9;      // slider variable connecetd to analog pin A9
const int JOYSTICK_1_SW_PIN = 32; // switch output connected to digital pin 32
const int JOYSTICK_2_1 = A10;     // slider variable connected to analog pin A10
const int JOYSTICK_2_2 = A11;     // slider variable connected to analog pin A11
const int JOYSTICK_2_SW_pin = 33; // switch output connected to digital pin 33
const int MATRIX_ROWS = 3;

// Base Parameters
const int LINKAGE_LENGTH = 90;   // s in matlab
const int SERVO_ARM_LENGTH = 24; // a in matlab
const double BASE_POSITIONS[6][3] = {
    {83.5, 33.4, 0.0},
    {-12.82, 89.01, 0.0},
    {-70.68, 55.61, 0.0},
    {-70.68, -55.61, 0.0},
    {-12.82, -89.01, 0.0},
    {83.5, -33.4, 0.0}};
const double PLATFORM_POSITIONS[6][3] = {
    {65.01, 6, 0.0},
    {-38.49, 66.67, 0.0},
    {-48.89, 60.67, 0.0},
    {-48.89, -60.67, 0.0},
    {-38.49, -66.67, 0.0},
    {66.6, -6, 0.0}};

const double BETA[6] = {0.0, 120.0, 120.0, 240.0, 240.0, 0.0};
const unsigned long UPDATE_INTERVAL = 66;

// Angles
double theta = 0.0;
double phi = 0.0;
double psi = 0.0;
double angles[3] = {0.0, 0.0, 0.0};

// Home height Vector (Also zt)
double home_height[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Runs (xp-xb)^2 and (yp-yb)^2
float base_platform_deltas[6][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
// indexed from motor 1-6
double alphas[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Reading
int gate_reading_1 = 0;
int gate_reading_2 = 0;
int gate_reading_3 = 0;
int gate_reading_4 = 0;
int gate_reading_5 = 0;
int gate_reading_6 = 0;
int joystick_reading_1_1 = 0;
int joystick_reading_1_2 = 0;
int joystick_reading_1_SW_pin = 0;
int joystick_reading_2_1 = 0;
int joystick_reading_2_2 = 0;
int joystick_reading_2_SW_pin = 0;
int joystick_angle[3] = {0, 0, 0};
// Message Timing
unsigned long lcd_msg_1[4] = {0, 0, 0, 0};
unsigned long lcd_msg_2[4] = {0, 0, 0, 0};
unsigned long update = 0;

int servo_settings[6] = {0, 0, 0, 0, 0, 0}; // PWM var
bool possible[6] = {true, true, true, true, true, true};

bool joystick_on = false;
bool keyboard_off = false;
bool self_solve_on = false;
// Initializing PWM Shield
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Initializing lCD
LiquidCrystal lcd(19, 18, 47, 49, 51, 53); // REGISTER SELECT PIN,ENABLE PIN,D4 PIN,D5 PIN, D6 PIN, D7 PIN

void setup()
{
    Serial.begin(9600);    // opens serial port, sets data rate to 9600 bps
    Serial.setTimeout(10); // change default (1000ms) to have faster response time
    pwm.begin();
    pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
    lcd.begin(16, 2);
    pinMode(JOYSTICK_1_SW_PIN, INPUT);     // SET PIN AS INPUT
    pinMode(JOYSTICK_2_SW_pin, INPUT);     // SET PIN AS INPUT
    digitalWrite(JOYSTICK_1_SW_PIN, HIGH); // ENABLE THE INTERNAL PULLUP ON THE INPUT PIN
    digitalWrite(JOYSTICK_2_SW_pin, HIGH); // ENABLE THE INTERNAL PULLUP ON THE INPUT PIN
    int mid_1 = 0;
    int mid_2 = 0;
    for (int i = 0; i < 6; i++)
    {
        // Changing the order of the servo settings array allows us to change the orientation of the maze if
        // we use delta angle changes for our servos
        mid_1 = (SERVOMINS[i] + SERVOMAXS[i]);
        mid_2 = mid_1 / 2;
        servo_settings[i] = mid_2;
        Serial.print(mid_2);
        Serial.print(",");
        base_platform_deltas[i][0] = pow((PLATFORM_POSITIONS[i][0] - BASE_POSITIONS[i][0]), 2); // (xp-xb)^2
        base_platform_deltas[i][1] = pow((PLATFORM_POSITIONS[i][1] - BASE_POSITIONS[i][1]), 2); // (yp-yb)^2
        // home_height[i] is also zt[i]
        home_height[i] = sqrt((LINKAGE_LENGTH * LINKAGE_LENGTH + SERVO_ARM_LENGTH * SERVO_ARM_LENGTH - base_platform_deltas[i][0] - base_platform_deltas[i][1] - PLATFORM_POSITIONS[i][2]));
    }
    Serial.println("");
    SetServos(servo_settings[0], servo_settings[1], servo_settings[2], servo_settings[3], servo_settings[4], servo_settings[5]);
    lcd_msg_1[0] = millis();
    lcd_msg_2[0] = millis() + LCD_CHANGE_DELAY / 2;
    update = millis();
    theta = 0.0;
    phi = 0.0;
    psi = 0.0;

    for (int i = 0; i < 3; i++)
    {
        angles[i] = 0.0;
    }
    ResetLCD();
    lcd.print("Lets get started");
    delay(1000);
}

void loop()
{
    if (millis() > lcd_msg_1[0] + LCD_CHANGE_DELAY && !joystick_on && joystick_reading_2_SW_pin)
    {
        lcd_msg_1[0] = millis();
        ResetLCD();
        lcd.print("Manual Control:");
        lcd.setCursor(0, 1);
        lcd.print("Click Joystick 1");
    }
    if (millis() > lcd_msg_2[0] + LCD_CHANGE_DELAY && !joystick_on && joystick_reading_2_SW_pin)
    {
        lcd_msg_2[0] = millis();
        ResetLCD();
        lcd.print("Self Solve:");
        lcd.setCursor(0, 1);
        lcd.print("Click Joystick 2");
    }
    JoysticksOn();
    ManualControl();
    if (!joystick_reading_2_SW_pin)
    {
        SolveMaze();
    }
}

void ServoValues()
{
    ResetLCD();
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
// TODO: INCREASE SERVO ANGLE INCREMENTALLY TO GUARD AGAINST WRONG DIRECTIVE
void SetServos(double motor_1, double motor_2, double motor_3, double motor_4, double motor_5, double motor_6)
{
    // temp array
    double temp_servo_settings[6] = {motor_1,
                                     motor_2,
                                     motor_3,
                                     motor_4,
                                     motor_5,
                                     motor_6};
    static int pwm_index = -1;

    Serial.print("Hello: ");
    for (int i = 0; i < 6; i++)
    {
        // Check that temp_servo_settings is not NULL and it is within our angle bounds
        // might need to change the bound of 0 -> 180 because sometimes the angle code outputs negative angles
        if (temp_servo_settings[i] >= -90 && temp_servo_settings[i] < 90)
        {
            servo_settings[i] = map(temp_servo_settings[i], -90, 90, SERVOMINS[i], SERVOMAXS[i]);
            Serial.print(servo_settings[i]);
            Serial.print(",");
        }
        pwm_index += 2;
        pwm.setPWM(pwm_index, 0, servo_settings[i]); // added +1 to match PWM port numbering (pins 1..6 used)
    }
    Serial.println("");
    pwm_index = -1;
}

void SolveMaze()
{
    // First gate
    ServoValues(); // Servo values are used for debugging
    SetServos(36.87, 7.34, 9.07, -31.36, 52.50, -26.40);
    ResetLCD();
    lcd.print("Heading to first");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    int gate_reading = analogRead(GATE_PIN_1); // Initialize gate reading for while loop
    // Maintain servos at required position using the while loop as a blocker
    // while (gate_reading < GATE_LIMIT)
    // {
    //     JoysticksOn();
    //     ReadAllPhotocells();
    //     gate_reading = analogRead(GATE_PIN_1);
    // }
    delay(1500);

    ServoValues(); // Servo values are used for debugging
    SetServos(44.37, 7.43, 24.72, -37.18, 37.18, -21.28);
    ResetLCD();
    lcd.print("Heading to first");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    gate_reading = analogRead(GATE_PIN_1);
    // Maintain servos at required position using the while loop as a blocker
    // while (gate_reading < GATE_LIMIT)
    // {
    //     JoysticksOn();
    //     ReadAllPhotocells();
    //     gate_reading = analogRead(GATE_PIN_1);
    // }
    delay(1500);

    // 24.05,-45.12,41.87,-5.95,3.82,-20.60,

    // Second gate
    ServoValues();
    // SetServos(27.63,-20.32,19.47,-19.47,20.32,-26.58);
    SetServos(51.46, 36.49, 0.84, -45.73, 45.73, -32.51);
    ResetLCD();
    lcd.print("Heading to second");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    gate_reading = analogRead(GATE_PIN_2);
    // while (gate_reading < GATE_LIMIT)
    // {
    //     JoysticksOn();
    //     ReadAllPhotocells();
    //     gate_reading = analogRead(GATE_PIN_2);
    // }
    delay(1500);

    ServoValues(); // Servo values are used for debugging
    SetServos(-6.02, -67.39, 74.32, -31.91, 22.22, 6.64);
    ResetLCD();
    lcd.print("Heading to first");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    gate_reading = analogRead(GATE_PIN_1);
    // Maintain servos at required position using the while loop as a blocker
    // while (gate_reading < GATE_LIMIT)
    // {
    //     JoysticksOn();
    //     ReadAllPhotocells();
    //     gate_reading = analogRead(GATE_PIN_1);
    // }
    delay(100);

    ServoValues();
    SetServos(21.03, -77.83, 47.77, -6.02, -12.80, -28.63);
    ResetLCD();
    lcd.print("Heading to second");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    gate_reading = analogRead(GATE_PIN_2);
    // while (gate_reading < GATE_LIMIT)
    // {
    //     JoysticksOn();
    //     ReadAllPhotocells();
    //     gate_reading = analogRead(GATE_PIN_2);
    // }
    delay(1500);
    // Third gate
    ServoValues();
    SetServos(22.90, -22.90, 22.90, 25.77, -25.55, -12.68);
    ResetLCD();
    lcd.print("Heading to third");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    gate_reading = analogRead(GATE_PIN_3);
    delay(1500);
    // while (gate_reading < GATE_LIMIT)
    // {
    //     JoysticksOn();
    //     ReadAllPhotocells();
    //     gate_reading = analogRead(GATE_PIN_3);
    // }

    // // Fourth gate
    // ServoValues();
    // SetServos(100, 100, 100, 500, 100, 100);
    // Serial.println("Heading to the fourth gate");
    // gate_reading = analogRead(GATE_PIN_4);
    // while (gate_reading < GATE_LIMIT)
    // {
    //     JoysticksOn();
    //     ReadAllPhotocells();
    //     gate_reading = analogRead(GATE_PIN_4);
    // }
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
    // Serial.println("Hellooo Worlslslslsdldlds");
    // index is w * h
    double rotation_matrix[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double inc[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double alpha_inc[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int c = 0; // Counter Variable

    JoysticksOn();

    angles[0] = DegToRad(psi);
    angles[1] = DegToRad(theta);
    angles[2] = DegToRad(phi);
    CalcRotation(rotation_matrix, angles[0], angles[1], angles[2]);

    for (int i = 0; i < 6; i++)
    {
        alphas[i] = ServoAngle(possible[i], BASE_POSITIONS[i][c], BASE_POSITIONS[i][(c + 1)], BASE_POSITIONS[i][c + 2], PLATFORM_POSITIONS[i][c], PLATFORM_POSITIONS[i][(c + 1)], PLATFORM_POSITIONS[i][c + 2], BETA[i], rotation_matrix, home_height[i]);
    }

    if (possible[0] && possible[1] && possible[2] && possible[3] && possible[4] && possible[5])
    {
        // alphas[1] = 90 - alphas[1];
        // alphas[3] = 90 - alphas[3];
        // alphas[5] = 90 - alphas[5];
        alphas[1] = -1 * alphas[1];
        alphas[3] = -1 * alphas[3];
        alphas[5] = -1 * alphas[5];
        SetServos(alphas[0], alphas[1], alphas[2], alphas[3], alphas[4], alphas[5]);
    }
    else
    {
        ResetLCD();
        lcd.print("Angle not");
        lcd.setCursor(0, 1);
        lcd.print("possible!");
    }
    Serial.print("Alphas: ");
    for (int i = 0; i < 6; i++)
    {

        Serial.print(alphas[i]);
        Serial.print(",");
    }
    Serial.println("");
    if (millis() > lcd_msg_1[1] + LCD_CHANGE_DELAY)
    {
        lcd_msg_1[1] = millis();
        ResetLCD();
        lcd.print("Servo Values:");
    }
    if (millis() > lcd_msg_2[1] + LCD_CHANGE_DELAY)
    {
        lcd_msg_2[1] = millis();
        ResetLCD();
        ServoValues();
    }
}

void ReadJoysticks()
{
    joystick_reading_1_1 = analogRead(JOYSTICK_1_1);
    delay(10);
    joystick_reading_1_2 = analogRead(JOYSTICK_1_2);
    delay(10);
    joystick_reading_1_SW_pin = digitalRead(JOYSTICK_1_SW_PIN);

    joystick_reading_2_1 = analogRead(JOYSTICK_2_1);
    delay(10);
    joystick_reading_2_2 = analogRead(JOYSTICK_2_2);
    delay(10);
    joystick_reading_2_SW_pin = digitalRead(JOYSTICK_2_SW_pin);
}

void JoysticksOn() 
{
    static double angle_max = 30.0;
    ReadJoysticks();

    // if (!joystick_reading_1_SW_pin) // joystick pin pressed down = 0
    // {
    //     self_solve_on = false;
    //     joystick_on = !joystick_on;
    //     Serial.println("AHAHAHAHAH");
    // }
    // if (!joystick_reading_2_SW_pin)
    // {
    //     joystick_on = false;
    //     self_solve_on = !self_solve_on;
    // }

    if (millis() > update + UPDATE_INTERVAL)
    {
        update = millis();

        if (joystick_reading_2_1 > 573 && psi <= angle_max)
        {
            psi += SERVOCHG * joystick_reading_2_1 / 300;
        }
        else if (joystick_reading_2_1 < 450 && psi >= -angle_max)
        {
            psi -= SERVOCHG * (1023 - joystick_reading_2_1) / 300;
        }
        if (joystick_reading_1_2 > 573 && phi <= angle_max)
        {
            phi += SERVOCHG * joystick_reading_1_2 / 300;
        }
        else if (joystick_reading_1_2 < 450 && phi >= -angle_max)
        {
            phi -= SERVOCHG * (1023 - joystick_reading_1_2) / 300;
        }
        if (joystick_reading_1_1 > 573 && theta <= angle_max)
        {
            theta += SERVOCHG;
        }
        else if (joystick_reading_1_1 < 450 && theta >= -angle_max)
        {
            theta -= SERVOCHG;
        }
    }
    Serial.println(theta);
}

// ServoAngles(&alphas, ) <- pass in variables as such
double ServoAngle(bool possible, double base_x, double base_y, double base_z, double plat_x, double plat_y, double plat_z, double Beta, double rot[], double home_height)
{
    double li[3] = {0.0, 0.0, 0.0};
    double qi[3] = {0.0, 0.0, 0.0};
    double base_to_center[3] = {0.0, 0.0, 0.0};
    double mult_result[3] = {0.0, 0.0, 0.0};
    double sum_result[3] = {0.0, 0.0, 0.0};
    double sub_result[3] = {0.0, 0.0, 0.0};
    double length_possible = 0.0, lsquared = 0.0, L = 0.0, M = 0.0, N = 0.0, alpha = 0.0;
    double base_coord[3] = {base_x, base_y, base_z};
    double platform_coord[3] = {plat_x, plat_y, plat_z};

    possible = true;
    base_to_center[2] = home_height;

    MatrixMultRotation(rot, platform_coord, mult_result);
    // Note, subtraction is not commutative, order matters
    MatrixSummation(mult_result, base_to_center, 1, 3, qi, true);
    MatrixSummation(qi, base_coord, 1, 3, li, false);

    for (int j = 0; j < 3; j++)
    {
        mult_result[j] = 0.0;
        sum_result[j] = 0.0;
        sub_result[j] = 0.0;
    }

    lsquared = ((qi[0] * qi[0]) + (qi[1] * qi[1]) + (qi[2] * qi[2])) + ((base_x * base_x) + (base_y * base_y) + (base_z * base_z)) - 2 * ((qi[0] * base_x) + (qi[1] * base_y) + (qi[2] * base_z));
    L = lsquared - ((LINKAGE_LENGTH * LINKAGE_LENGTH) - (SERVO_ARM_LENGTH * SERVO_ARM_LENGTH));
    M = 2 * SERVO_ARM_LENGTH * (qi[2] - base_z);
    N = 2 * SERVO_ARM_LENGTH * ((cos(DegToRad(Beta)) * (qi[0] - base_x)) + (sin(DegToRad(Beta)) * (qi[1] - base_y)));

    // Check whether servo angles are possible
    length_possible = L / (sqrt((M * M + N * N)));
    if (abs(length_possible) >= 1)
    {
        possible = false;
        return;
    }
    alpha = RadToDeg((asin(length_possible) - atan((N / M))));
    return alpha;
}

double DegToRad(double deg)
{
    double rad = 0.0;
    rad = (deg * 1000.0) / 57296.0;
    return rad;
}

double RadToDeg(double rad)
{
    double deg = 0.0;
    deg = (rad * 57296.0) / 1000.0;
    return deg;
}

// Make sure angles are in radians
void CalcRotation(double rot[], double psi, double theta, double phi)
{
    // index as width * row + col
    rot[(3 * 0 + 0)] = cos(psi) * cos(theta);
    rot[(3 * 0 + 1)] = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
    rot[(3 * 0 + 2)] = sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi);
    rot[(3 * 1 + 0)] = sin(psi) * cos(theta);
    rot[(3 * 1 + 1)] = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
    rot[(3 * 1 + 2)] = -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi);
    rot[(3 * 2 + 0)] = -sin(theta);
    rot[(3 * 2 + 1)] = cos(theta) * sin(phi);
    rot[(3 * 2 + 2)] = cos(theta) * cos(phi);
}

void MatrixMultRotation(double rot[], double platform[], double result[])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i] += rot[(3 * i + j)] * platform[j];
        }
    }
}

void MatrixSummation(double M1[], double M2[], int rows, int col, double results[], bool sum)
{
    if (sum)
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < col; j++)
            {
                results[((col + 1) * i + j)] = M1[((col + 1) * i + j)] + M2[((col + 1) * i + j)];
            }
        }
    }
    else
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < col; j++)
            {
                results[((col + 1) * i + j)] = M1[((col + 1) * i + j)] - M2[((col + 1) * i + j)];
            }
        }
    }
}

void ServoMiddle()
{
    int mid_1 = 0;
    int mid_2 = 0;
    theta = 0.0;
    phi = 0.0;
    psi = 0.0;
    for (int i = 0; i < 6; i++)
    {
        mid_1 = (SERVOMINS[i] + SERVOMAXS[i]);
        mid_2 = mid_1 / 2;
        servo_settings[i] = mid_2;
    }
    ServoValues();
    SetServos(servo_settings[0], servo_settings[1], servo_settings[2], servo_settings[3], servo_settings[4], servo_settings[5]);
}

void ResetLCD()
{
    lcd.clear();
    lcd.setCursor(0, 0);
}
