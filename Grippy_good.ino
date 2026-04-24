// AL5D Robot Arm Controller for Arduino Nano ESP32
// Button-based gripper control (no encoder)

// BUTTON PINS (Change if needed)
const int BTN_OPEN  = D2;
const int BTN_CLOSE = D3;
const int BTN_SPEED = D4;

bool fastMode = false;   // start in slow mode
int gripStepSlow = 8;    // small open/close
int gripStepFast = 25;   // fast open/close

// Joystick pins
const int left_SW_pin  = D7;
const int left_x_pin   = A0;
const int left_y_pin   = A1;
const int right_SW_pin = D8;
const int right_x_pin  = A7;
const int right_y_pin  = A6;

// Offsets for joystick deadzones
int left_base_y, left_base_x;
int right_base_y, right_base_x;

// Servo pulse arrays
uint16_t pulses[6] = {1500, 2000, 2000, 800, 1500, 1500};

// Home position string for reset


void setup()
{
    pinMode(BTN_OPEN,  INPUT_PULLUP);
    pinMode(BTN_CLOSE, INPUT_PULLUP);
    pinMode(BTN_SPEED, INPUT_PULLUP);

    pinMode(left_SW_pin, INPUT_PULLUP);
    pinMode(right_SW_pin, INPUT_PULLUP);

    Serial.begin(9600);
    delay(2000);

    Serial.println("Connecting to SSC-32U...");
    Serial1.begin(9600, SERIAL_8N1, D12, D10); // RX = D12, TX = D10 SO D12 GOES TO TX ON THE ARM
    delay(100);
    Serial.println(" Done");

    Serial.print("Calibrating Joysticks...");
    int samples = 20;
    left_base_y = left_base_x = right_base_y = right_base_x = 0;

    for(int i = 0; i < samples; i++)
    {
        left_base_y  += analogRead(left_y_pin);
        left_base_x  += analogRead(left_x_pin);
        right_base_y += analogRead(right_y_pin);
        right_base_x += analogRead(right_x_pin);
        delay(10);
    }

    left_base_y  /= samples;
    left_base_x  /= samples;
    right_base_y /= samples;
    right_base_x /= samples;

    Serial.println(" Done");
    Serial.println("Ready.\n");

    // Initialize Home position with current pulses
    // sprintf(HomePos, "#0P%u #1P%u #2P%u #3P%u #4P%u #5P%u T2000\r",
    //         pulses[0], pulses[1], pulses[2], pulses[3], pulses[4], pulses[5]);
}

void loop()
{
   const int DEADZONE = 400;
int gripStep = 80;  // stays fixed as requested

// === BUTTON SPEED TOGGLE LOGIC ===
static bool prevSpeedState = HIGH;
bool currentSpeedState = digitalRead(BTN_SPEED);

if(currentSpeedState == LOW && prevSpeedState == HIGH)
{
    fastMode = !fastMode;   // toggle
    delay(200);             // debounce
}
prevSpeedState = currentSpeedState;

// Speed affects joint motion only (not gripper)
float SPEED = 0.8;
float SPEEDBASE = fastMode ? 4.0 : 0.8;



    // === BUTTON GRIPPER CONTROL ===
    if(digitalRead(BTN_OPEN)  == LOW) pulses[5] += gripStep;
    if(digitalRead(BTN_CLOSE) == LOW) pulses[5] -= gripStep;

    // === JOYSTICK READING ===
    int lx = analogRead(left_x_pin);
    int ly = analogRead(left_y_pin);
    int rx = analogRead(right_x_pin);
    int ry = analogRead(right_y_pin);

    int diff;
    diff = ry - right_base_y; if(abs(diff) > DEADZONE) pulses[0] -= (diff * SPEEDBASE) / 120.0; // Base
    diff = rx - right_base_x;  if(abs(diff) > DEADZONE) pulses[1] -= (diff * SPEED) / 120.0; // Shoulder
    diff = lx - left_base_x; if(abs(diff) > DEADZONE) pulses[2] += (diff * SPEED) / 120.0; // Elbow
    diff = ly - left_base_y;  if(abs(diff) > DEADZONE) pulses[3] += (diff * SPEED) / 120.0; // Wrist

    // === JOYSTICK BUTTONS FOR WRIST ROTATION AND HOME RESET WITH EDGE DETECTION ===
    static bool prevLeftSW  = HIGH;
    static bool prevRightSW = HIGH;

    bool curLeftSW  = digitalRead(left_SW_pin);
    bool curRightSW = digitalRead(right_SW_pin);

    // Home reset triggers only on press transition
   // Home reset triggers only on press transition
if(curLeftSW == LOW && curRightSW == LOW && (prevLeftSW == HIGH || prevRightSW == HIGH))
{
    // Send the default pulse values
    char homeOutput[100];
    sprintf(homeOutput, "#0P%u #1P%u #2P%u #3P%u #4P%u T2000\r",
            1500, 2000, 1800, 800, 1500);  // default pulse values
    Serial1.write(homeOutput);

    // Reset current pulses to default so joystick/gripper continue from home
    pulses[0] = 1500;
    pulses[1] = 2000;
    pulses[2] = 2000;
    pulses[3] = 800;
    pulses[4] = 1500;
  
}


    prevLeftSW  = curLeftSW;
    prevRightSW = curRightSW;

    // === CLAMP PULSES TO SAFE RANGE ===
    pulses[0] = constrain(pulses[0], 600, 2400);   // Base
    pulses[1] = constrain(pulses[1], 1200, 2200);  // Shoulder
    pulses[2] = constrain(pulses[2], 1000, 2200);  // Elbow
    pulses[3] = constrain(pulses[3], 500, 2300);   // Wrist
    pulses[4] = constrain(pulses[4], 800, 2200);   // Wrist Rotate
    pulses[5] = constrain(pulses[5], 1100, 1900);  // Gripper

    // === SEND COMMANDS TO SSC-32U ===
    char output[100];
    sprintf(output, "#0P%u #1P%u #2P%u #3P%u #4P%u #5P%u T60\r",
            pulses[0], pulses[1], pulses[2], pulses[3], pulses[4], pulses[5]);
    Serial1.write(output);

    delay(30);
}
