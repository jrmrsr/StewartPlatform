/* =================================================================================================== 
 * AUTOMATED SERVO CODE V1 
 =================================================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal.h>

const int SERVOMINS[6] = {150, 150, 150, 150, 150, 150};
const int SERVOMAXS[6] = {500, 500, 500, 500, 500, 500};
const int LCD_CHANGE_DELAY = 4000;
const int GATE_LIMIT = 450;
const int MATRIX_ROWS = 3;
const double SERVOCHG = 1.0;

// PIN ASSIGNMENTS //
const int GATE_PIN_1 = A7;
const int GATE_PIN_2 = A12;
const int GATE_PIN_3 = A13;
const int GATE_PIN_4 = A14;
const int GATE_PIN_5 = A15;
const int GATE_PIN_6 = A0;
const int JOYSTICK_1_1 = A8;
const int JOYSTICK_1_2 = A9;
const int JOYSTICK_1_SW_PIN = 38;
const int JOYSTICK_2_1 = A10;
const int JOYSTICK_2_2 = A11;
const int JOYSTICK_2_SW_PIN = 39;
const int LCD_CONTRAST_PIN = A2;

// BASE AND PLATFORM DISTANCES //
const int LINKAGE_LENGTH = 90;   // s in matlab
const int SERVO_ARM_LENGTH = 24; // a in matlab
const int UPDATE_INTERVAL = 50;
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

// PLATFORM COORDINATE ANGLES //
double theta = 0.0;
double phi = 0.0;
double psi = 0.0;
double angles[3] = {0.0, 0.0, 0.0};
double home_height[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // zt in matlab code
double alphas[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      // Indexed from motors 1-6
float base_platform_deltas[6][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

// ANALOG AND DIGITAL READINGS //
int gate_readings[6] = {0, 0, 0, 0, 0, 0};
int joystick_reading_1_1 = 0;
int joystick_reading_1_2 = 0;
int joystick_reading_1_SW_pin = 0;
int joystick_reading_2_1 = 0;
int joystick_reading_2_2 = 0;
int joystick_reading_2_SW_pin = 0;
int joystick_angle[3] = {0, 0, 0};
int mid_1 = 0;
int mid_2 = 0;

// LCD AND RUN TIMERS //
unsigned long lcd_msg_1[2] = {0, 0};
unsigned long lcd_msg_2[2] = {0, 0};
unsigned long update = 0;
unsigned long run_time = 0;

// SERVO ANGLES/PWM VALUES //
int servo_settings[6] = {0, 0, 0, 0, 0, 0};
bool possible_angle[6] = {true, true, true, true, true, true};

// BOOLEAN LCD FLAGS FOR TIMERS //
bool manual_solve_lcd = true;
bool manual_solve_on = false;

// INITIALIZE PWM //
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// INITIALIZE lCD //
LiquidCrystal lcd(19, 18, 30, 28, 26, 24); // REGISTER SELECT PIN, ENABLE PIN, D4 PIN, D5 PIN, D6 PIN, D7 PIN

void setup()
{
    Serial.begin(9600);    // Opens serial port, sets data rate to 9600 bps
    Serial.setTimeout(10); // Default response time

    pwm.begin();
    pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates

    lcd.begin(16, 2);

    pinMode(JOYSTICK_1_SW_PIN, INPUT); // Set joystick click pins as inputs
    pinMode(JOYSTICK_2_SW_PIN, INPUT);
    pinMode(LCD_CONTRAST_PIN, OUTPUT); // Set contrast pin as output

    digitalWrite(JOYSTICK_1_SW_PIN, HIGH); // Enable the internal pullup on the input pin
    digitalWrite(JOYSTICK_2_SW_PIN, HIGH);
    analogWrite(LCD_CONTRAST_PIN, 5); // Set the LCD contrast

    for (int i = 0; i < 6; i++)
    {
        base_platform_deltas[i][0] = pow((PLATFORM_POSITIONS[i][0] - BASE_POSITIONS[i][0]), 2); // (xp-xb)^2 from matlab
        base_platform_deltas[i][1] = pow((PLATFORM_POSITIONS[i][1] - BASE_POSITIONS[i][1]), 2); // (yp-yb)^2 from matlab
        home_height[i] = sqrt((LINKAGE_LENGTH * LINKAGE_LENGTH + SERVO_ARM_LENGTH * SERVO_ARM_LENGTH - base_platform_deltas[i][0] - base_platform_deltas[i][1] - PLATFORM_POSITIONS[i][2]));
    }

    ResetLCD();
    lcd.print("Automate Maze");
    lcd.setCursor(0, 1);
    lcd.print("Solver V1");

    manual_solve_on = false;

    // SET LCD MESSAGE TIMERS //
    lcd_msg_1[0] = millis() + 1000; // Add 1s to keep the start up message on the screen without blocking the joysticks from working
    lcd_msg_2[0] = millis() + 1000 + (LCD_CHANGE_DELAY / 2);
    update = millis(); // Joystick to platform update timer; this will slow down the joystick to reduce cross talk and make it easier to solve manually
}

void loop()
{
    if (!manual_solve_on) // Constant check to prevent the LCD manual timer message from occurring on every loop
    {
        run_time = 0;
    }

    // LCD ALTERNATING MESSAGES //
    if (millis() > lcd_msg_1[0] + LCD_CHANGE_DELAY && joystick_reading_2_SW_pin)
    {
        lcd_msg_1[0] = millis();
        ResetLCD();
        lcd.print("Timer Solve:");
        lcd.setCursor(0, 1);
        lcd.print("Click Joystick 1");
    }
    if (millis() > lcd_msg_2[0] + LCD_CHANGE_DELAY && joystick_reading_2_SW_pin)
    {
        lcd_msg_2[0] = millis();
        manual_solve_lcd = !manual_solve_lcd;
        if (manual_solve_lcd)
        {
            ResetLCD();
            lcd.print("Manual Solve:");
            lcd.setCursor(0, 1);
            lcd.print("Use Joysticks");
        }
        else
        {
            ResetLCD();
            lcd.print("Sensor Solve:");
            lcd.setCursor(0, 1);
            lcd.print("Click Joystick 2");
        }
    }

    gate_readings[5] = analogRead(GATE_PIN_6);                             // Always reading photocell at the last gate to check whether a user solve the maze manually
    if (gate_readings[5] < GATE_LIMIT && run_time != 0 && manual_solve_on) // Once the user solves the maze, display run time message on LCD
    {
        lcd_msg_1[0] = millis() + 1000;
        lcd_msg_2[0] = millis() + 1000 + LCD_CHANGE_DELAY / 2;
        run_time = millis() - run_time;
        ResetLCD();
        lcd.print("Manual Solve");
        lcd.setCursor(0, 1);
        lcd.print("Time: ");
        lcd.print(run_time);
        manual_solve_on = false;
        run_time = 0;
    }

    JoystickSetAngle(); // Read joysticks for input
    ManualControl();    // Solve for maze position based on input

    // AUTOMATED SOLVERS //
    if (!joystick_reading_2_SW_pin)
    {
        MazeSolverSensors();
    }
    if (!joystick_reading_1_SW_pin)
    {
        MazeSolverTimer();
    }
}

// SERVO HELPER FUNCTIONS //
/* SetServos: Anything above 90 will be treated as a straight PWM value, such that this function can be used with both PWM values and angles
   as inputs. For instance, the SetServoMiddle function writes the midpoint PWM value to the servos, not 0 degrees. This is done for accuracy 
   and to test the actual PWM bounds, as they are often drifting. Most of the time, the function will be receiving angles in degrees from the 
   matlab code adaptation, hence the variable naming. */
void SetServos(double angle_motor_1, double angle_motor_2, double angle_motor_3, double angle_motor_4, double angle_motor_5, double angle_motor_6)
{
    double temp_servo_settings[6] = {angle_motor_1,
                                     angle_motor_2,
                                     angle_motor_3,
                                     angle_motor_4,
                                     angle_motor_5,
                                     angle_motor_6};

    for (int i = 0; i < 6; i++)
    {
        if (temp_servo_settings[i] >= -90 && temp_servo_settings[i] < 90) // -90 -> 90 is 180 degrees of motion, which is the maximum for our servos
        {
            servo_settings[i] = map(temp_servo_settings[i], -90, 90, SERVOMINS[i], SERVOMAXS[i]);
        }
        pwm.setPWM(i + 1, 0, servo_settings[i]); // added +1 to match PWM port numbering (pins 1..6 used)
    }
}

/* Sets each servo to its respective midpoint PWM value. It also resets the roll, pitch, and yaw of the top platform. */
void SetServoMiddle()
{
    mid_1 = 0, mid_2 = 0;
    theta = 0.0, phi = 0.0, psi = 0.0;

    for (int i = 0; i < 6; i++)
    {
        mid_1 = (SERVOMINS[i] + SERVOMAXS[i]);
        mid_2 = mid_1 / 2;
        servo_settings[i] = mid_2;
    }
    SetServos(servo_settings[0], servo_settings[1], servo_settings[2], servo_settings[3], servo_settings[4], servo_settings[5]);
}

// AUTOMATED SOLVERS //
/* MazeSolverSensors(): This function will solve the maze using the photocells placed under the platform. It sets the platform 
   to the desired motor angles, and waits on the gate readings to change its orientation. It will only ever read from one gate
   at a time, to prevent any issues between different sensor readings. There are a few notable delays sprinkled in, which have
   increased the reliability of the solver given the small number of sensors. It also has 2 timeouts, one to deal with a common
   catch point in the maze, where it will flick the ball out of where it got caught. The second will cause the platform to move 
   on from trying to solve itself if it takes too long to solve, and returns into the loop function. */
void MazeSolverSensors()
{
    run_time = millis();

    SetServos(19.65, 16.51, -15.66, -71.23, 83.32, -25.28);
    ResetLCD();
    lcd.print("Heading to first");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    gate_readings[0] = analogRead(GATE_PIN_1);
    while (gate_readings[0] > GATE_LIMIT)
    {
        gate_readings[0] = analogRead(GATE_PIN_1);
    }

    SetServos(51.46, 36.49, 0.84, -45.73, 45.73, -32.51);
    ResetLCD();
    lcd.print("Heading to ");
    lcd.setCursor(0, 1);
    lcd.print("second gate");
    gate_readings[2] = analogRead(GATE_PIN_3);
    while (gate_readings[2] > GATE_LIMIT)
    {
        gate_readings[2] = analogRead(GATE_PIN_3);
    }
    delay(750);

    SetServos(-6.02, -67.39, 74.32, -31.91, 22.22, 6.64);
    ResetLCD();
    lcd.print("Heading to third");
    lcd.setCursor(0, 1);
    lcd.print("gate");
    delay(100);

    SetServos(21.03, -77.83, 47.77, -6.02, -12.80, -28.63);
    delay(200);

    SetServos(29.12, -29.12, 72.33, 20.95, -21.75, -20.61);
    ResetLCD();
    lcd.print("Heading to ");
    lcd.setCursor(0, 1);
    lcd.print("fourth gate");
    gate_readings[4] = analogRead(GATE_PIN_5);
    while (gate_readings[4] > 550)
    {
        gate_readings[4] = analogRead(GATE_PIN_5);
        if ((millis() - run_time) > 5000)
        {
            gate_readings[4] = 0;

            SetServos(84.71, -25.86, 13.02, 41.69, -43.70, -74.35);
            delay(300);

            SetServos(29.12, -29.12, 72.33, 20.95, -21.75, -20.61);
            delay(250);
        }
    }

    SetServos(57.08, -38.96, 27.80, 29.58, -30.41, -50.21);
    ResetLCD();
    lcd.print("Heading to ");
    lcd.setCursor(0, 1);
    lcd.print("finish line");

    SetServos(22.90, -22.90, 22.90, 25.77, -25.55, -12.68);
    delay(1000);

    gate_readings[5] = analogRead(GATE_PIN_6);
    while (gate_readings[5] > GATE_LIMIT)
    {
        gate_readings[5] = analogRead(GATE_PIN_6);
        if ((millis() - run_time) > 15000)
        {
            gate_readings[5] = 0;
        }
    }

    run_time = millis() - run_time;
    ResetLCD();
    lcd.print("Run Time:");
    lcd.setCursor(0, 1);
    lcd.print(run_time);

    run_time = 0;
    lcd_msg_1[0] = millis() + 1000 + LCD_CHANGE_DELAY / 2; // Reset message timers
    lcd_msg_2[0] = millis() + 1000;
}

/* MazeSolverTimer(): This function will solve the maze using delays. This was dialed in as a fallback if the photocells
   stopped working. It is much simpler than the above example, and is also able to solve the maze. */
void MazeSolverTimer()
{
    run_time = millis();

    ResetLCD();
    lcd.print("Solving maze");
    lcd.setCursor(0, 1);
    lcd.print("with timer");

    SetServos(24.16, 24.34, -23.48, -68.29, 86.91, -30.62);
    delay(1000);

    SetServos(72.61, 39.87, -38.69, -12.01, 24.26, -84.87);
    delay(1500);

    SetServos(21.33, -43.58, 42.91, -10.01, 8.83, -17.18);
    delay(100);

    SetServos(9.77, -0.16, 1.87, -75.16, 73.91, -13.46);
    delay(80);

    SetServos(27.55, -76.08, 66.41, 15.14, -16.02, -19.77);
    delay(400);

    SetServos(22.90, -22.90, 22.90, 25.77, -25.55, -12.68);
    delay(1000);

    SetServos(49.65, -25.51, 18.98, 10.33, -9.00, -45.97);
    delay(1000);

    SetServos(43.62, -40.99, 33.21, 15.51, -15.58, -38.04);
    delay(700);

    gate_readings[5] = analogRead(GATE_PIN_6);
    while (gate_readings[5] > GATE_LIMIT)
    {
        gate_readings[5] = analogRead(GATE_PIN_6);
        if ((millis() - run_time) > 15000)
        {
            gate_readings[5] = 0;
        }
    }

    run_time = millis() - run_time;

    ResetLCD();
    lcd.print("Run Time:");
    lcd.setCursor(0, 1);
    lcd.print(run_time);

    run_time = 0;
    lcd_msg_1[0] = millis() + 1000 + LCD_CHANGE_DELAY / 2;
    lcd_msg_2[0] = millis() + 1000;
}

// MATLAB INVERSE KINEMATICS // 
void ManualControl()
{
    double rotation_matrix[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double inc[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    JoystickSetAngle(); 

    angles[0] = DegToRad(psi);
    angles[1] = DegToRad(theta);
    angles[2] = DegToRad(phi);
    CalcRotation(rotation_matrix, angles[0], angles[1], angles[2]);

    for (int i = 0; i < 6; i++)
    {
        /* Function for Alpha is specific to each motor's alpha, and requires the distance vectors of each motor. */
        alphas[i] = ServoAngle(possible_angle[i], BASE_POSITIONS[i][0], BASE_POSITIONS[i][(1)], BASE_POSITIONS[i][2], PLATFORM_POSITIONS[i][0], PLATFORM_POSITIONS[i][(1)], PLATFORM_POSITIONS[i][2], BETA[i], rotation_matrix, home_height[i]);
    }

    if (possible_angle[0] && possible_angle[1] && possible_angle[2] && possible_angle[3] && possible_angle[4] && possible_angle[5])
    {
        alphas[1] = -1 * alphas[1]; // Every other angle must be negative to output the correct motor positions, based on our platform design.
        alphas[3] = -1 * alphas[3];
        alphas[5] = -1 * alphas[5];
        SetServos(alphas[0], alphas[1], alphas[2], alphas[3], alphas[4], alphas[5]);
    }

    Serial.print("Alphas: ");
    for (int i = 0; i < 6; i++)
    {

        Serial.print(alphas[i]);
        Serial.print(",");
    }
    Serial.println("");
}

// JOYSTICK HELPER FUNCTIONS //
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
    joystick_reading_2_SW_pin = digitalRead(JOYSTICK_2_SW_PIN);
}

/*  JoystickSetAngle(): Sets angle input for manual control. The magnitude of the angle increases changes base on 
    how far you push the joystick. The further the joystick is pushed, the larger the change. 
    This helps with fine motor control for any tricky maneuvers. This function also implements
    a dead zone to make the motor reactions less jitery. */
void JoystickSetAngle()
{
    static double angle_max = 15.0;
    ReadJoysticks();

    if (millis() > update + UPDATE_INTERVAL) // Must rate limit the angle updates so as to make manual mode usable
    {
        update = millis();

        if (joystick_reading_2_1 > 573 && psi <= angle_max)
        {
            psi += SERVOCHG * joystick_reading_2_1 / 300;
            manual_solve_on = true;
        }
        else if (joystick_reading_2_1 < 450 && psi >= -angle_max)
        {
            psi -= SERVOCHG * (1023 - joystick_reading_2_1) / 300;
            manual_solve_on = true;
        }

        if (joystick_reading_1_2 > 573 && phi <= angle_max)
        {
            phi += SERVOCHG * joystick_reading_1_2 / 300;
            manual_solve_on = true;
        }
        else if (joystick_reading_1_2 < 450 && phi >= -angle_max)
        {
            phi -= SERVOCHG * (1023 - joystick_reading_1_2) / 300;
            manual_solve_on = true;
        }

        if (joystick_reading_1_1 > 573 && theta <= angle_max)
        {
            theta += SERVOCHG * joystick_reading_2_1 / 300;
            manual_solve_on = true;
        }
        else if (joystick_reading_1_1 < 450 && theta >= -angle_max)
        {
            theta -= SERVOCHG * (1023 - joystick_reading_1_2) / 300;
            manual_solve_on = true;
        }
    }
    if (run_time == 0 && manual_solve_on)
    {
        run_time = millis();
    }
}

// MATRIX MATH //
/* ServoAngle(): Essentially a one-to-one adaptation of the matlab code. it requires a lot of vectors and computes a lot of different, necessary values.
   Unfortunately, there was not a lot of time to fully optimize this code, but I did as much as I could within the timeframe. The Arduino MEGA was able 
   handle this without any issues and the platform was very responsive thankfully. Ideally I would not have to instantiate every variable each time this
   code runs, but the values must start as shown, otherwise the calculated angles would be useless. The matrix math was rather sensitive.*/ 
double ServoAngle(bool possible, double base_x, double base_y, double base_z, double plat_x, double plat_y, double plat_z, double beta, double rot[], double home_height)
{
    double length_possible = 0.0, lsquared = 0.0, L = 0.0, M = 0.0, N = 0.0, alpha = 0.0;

    double li[3] = {0.0, 0.0, 0.0};
    double qi[3] = {0.0, 0.0, 0.0};
    double base_to_center[3] = {0.0, 0.0, 0.0};
    double mult_result[3] = {0.0, 0.0, 0.0};
    double sum_result[3] = {0.0, 0.0, 0.0};
    double sub_result[3] = {0.0, 0.0, 0.0};
    double base_coord[3] = {base_x, base_y, base_z};
    double platform_coord[3] = {plat_x, plat_y, plat_z};

    possible = true;
    base_to_center[2] = home_height;

    MatrixMultRotation(rot, platform_coord, mult_result);
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
    N = 2 * SERVO_ARM_LENGTH * ((cos(DegToRad(beta)) * (qi[0] - base_x)) + (sin(DegToRad(beta)) * (qi[1] - base_y)));

    length_possible = L / (sqrt((M * M + N * N))); // Check whether servo angles are possible
    if (abs(length_possible) >= 1)
    {
        possible = false;
        return;
    }
    alpha = RadToDeg((asin(length_possible) - atan((N / M))));
    return alpha;
}

// MATH FUNCTIONS //
double DegToRad(double deg)
{
    return ((deg * 1000.0) / 57296.0);
}

double RadToDeg(double rad)
{
    return ((rad * 57296.0) / 1000.0);
}

/* CalcRotation(): Rotation matrix for motor angles. Note that the 2-D matrix was adapted to be one dimensional
   using the width * row + col indexing scheme. There were a lot of headaches with using a 2-D array and this was
   by far the simplest and most expedient method to deliver the right results. */
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

/* MatrixMultRotation(): This is not a general matrix multlipication, but rather a streamlined function for 
   this application. This can be done because the size of the possible ixj matrices is constant for the 
   Stewart platform. */
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

/* MatrixSummation(): This _is_ a general matrix summation and subtraction function, as both will be needed 
   for this application. As such, this function takes in the size of the ixj matrix. */
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

// LCD HELPER FUNCTIONS //
void ResetLCD() // Just for quality of life improvements
{
    lcd.clear();
    lcd.setCursor(0, 0);
}
