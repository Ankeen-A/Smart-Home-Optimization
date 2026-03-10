// LIBRARIES
#include <DHT.h>
#include <Servo.h>
#include <float.h>
#include <math.h>
#include <Fuzzy.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

// GLOBAL CONFIG & CONSTANTS
// System state
enum SystemMode {
  MODE_NORMAL_OPERATION, MODE_SET_TEMPERATURE, MODE_SET_WASHER_TIME,
  MODE_SET_WASHER_SOIL, MODE_SET_WASHER_FABRIC, MODE_SET_WASHER_LOAD
};
SystemMode current_mode = MODE_NORMAL_OPERATION;

// Pins
const int pin_temp_in     = 2;
const int pin_pot_tout    = A3;    // 
const int pin_photosensor = A2;    // Light sensor 0..1023
const int pin_led         = 3;     // LED PWM
const int pin_fan         = 5;     // Fan PWM
const int pin_servo       = 9;     // Blinds servo
const int pin_rgb_red     = 6;
const int pin_rgb_green   = 10;
const int pin_rgb_blue    = 11;
const int pin_ir_receiver = 12;

// IR codes
const unsigned long REMOTE_CODE_0    = 0xE619FF00;
const unsigned long REMOTE_CODE_1    = 0xBA45FF00;
const unsigned long REMOTE_CODE_2    = 0xB946FF00;
const unsigned long REMOTE_CODE_3    = 0xB847FF00;
const unsigned long REMOTE_CODE_4    = 0xBB44FF00;
const unsigned long REMOTE_CODE_5    = 0xBF40FF00;
const unsigned long REMOTE_CODE_6    = 0xBC43FF00;
const unsigned long REMOTE_CODE_7    = 0xF807FF00;
const unsigned long REMOTE_CODE_8    = 0xEA15FF00;
const unsigned long REMOTE_CODE_9    = 0xF609FF00;
const unsigned long REMOTE_CODE_STAR = 0xE916FF00;
const unsigned long REMOTE_CODE_HASH = 0xF20DFF00;
const unsigned long REMOTE_CODE_OK   = 0xE31CFF00;

// Lighting / blinds coupling -
const float B_CLOSED_MAX    = 0.20f;   // ≤20% open = "closed"
const float LED_MIN_CLOSED  = 0.07f;   // min LED when blinds are closed

// Lux target
const float lux_target       = 400.0f; // 
const float LUX_DEADBAND     = 15.0f;
const float LUX_HARD_PENALTY = 10000.0f; // huge penalty if lux unmet

// Night behavior
const int   NIGHT_RAW_THRESH  = 150;   // tune for sensor
const float LED_MIN_NIGHT     = 0.75f; // ≥75% at night
const float LED_NIGHT_TARGET  = 0.90f; // force 90% at night

// Constants
const float P_led_max       = 250.0f;   // W at l=1 calculated by 37sqm x 500lum/sqm / 90watt/lum = 205
const float shgc            = 0.5f;    //aka solar heat gain coeeficient
const float A_room          = 36.0f;   // consider 6x6
const float A_win           = 6.0f;    // standard 22%
const float A_walls         = 54.0f;   // consider ceiling height is 2.5 ->(6+6+6+6)*2.5 - 8 
const float A_total         = 124.0f;  // 36*2 + 52 
const float t_closed        = 0.15f;   // blinds transmission when closed aka 15% of light goes in when blinds fully closed
const float R_reflectance   = 0.5f;
const float U_win           = 2.0f;
const float U_walls         = 0.5f;
const float le_sun          = 0.01f;   // proxy
const float le_led          = 0.01f;   // proxy
const float led_heat        = 0.3f;    // proxy
const float E_led_max       = 486.0f;  // LED lux at l=1
const float COP             = 3.0f;    // AC COP
const float P_fan_max       = 100.0f;  // Estimated Max Fan Power for indoor unit (Watts)
const float tau_glass       = 0.60f;   // glass transmittance
const float FAN_ERR_FULL = 4.0f;   // fan linear control °C above setpoint ⇒ 100% fan

// LED perceived lux curve
const float LED_LUX_EFF   = 1.00f; // scale
const float LED_LUX_GAMMA = 0.80f; // l^gamma

// Blinds servo travel (tightened)
// Flip BLINDS_INVERT if direction is opposite.
const int  BLINDS_MIN_DEG = 40;   // 0% position
const int  BLINDS_MAX_DEG = 180;  // 100% position
const bool BLINDS_INVERT  = true; // true = 0% -> MAX, 100% -> MIN

// ----------------------------------------------------------------------------------
#define DHTTYPE DHT11
DHT sensorIndoor(pin_temp_in, DHTTYPE);
Servo blinds_servo;
LiquidCrystal_I2C lcd(0x27, 20, 4);

Fuzzy* fuzzy = new Fuzzy();
FuzzyInput* fuzzy_logic_input;
FuzzyOutput* washingCycle;

// Climate state
float T_in_set = 21.0f;

// Cycle setup
const unsigned long OPTIMIZATION_INTERVAL = 10000;  // 10 s
unsigned long lastRefreshTime = 0;
const unsigned long SENSOR_UI_INTERVAL = 100;       // 100 ms
unsigned long lastSensorUiTime = 0;

// Actuators (applied)
float b_applied = 0.50f;   // blinds 0..1
float l_applied = 0.00f;   // LEDs 0..1
int   fan_pwm_last = 0;    // 0..255

// Sensors (instant)
int   light_raw_inst = 0;

// Hysteresis
const float TEMP_DEADBAND = 0.3f;

// Washer state
long user_input_buffer = 0;
int washer_input_time = 0;     // HHMM 24h
int washer_input_soil = 0;     // 1..3
int washer_input_fabric = 0;   // 1..4 (VSoft..Hard)
int washer_input_load = 0;     // 1..3 (Small..Large)
// Low electricity windows (minutes since 00:00)
// 10:00–13:00 and 17:00–20:00
const int MORNING_START_MIN = 10*60, MORNING_END_MIN = 13*60;
const int EVENING_START_MIN = 17*60, EVENING_END_MIN = 20*60;

// ----------------------------------------------------------------------------------
// Clamp
static inline float clamp(float v, float lo, float hi) { return (v < lo ? lo : (v > hi ? hi : v)); }

// Fixed-width 20-char LCD line
void lcdPrint20S(int row, const String &s) {
  char line[21]; for (int i=0;i<20;i++) line[i] = ' '; line[20] = '\0';
  const char* c = s.c_str();
  for (int i=0; i<20 && c[i]; i++) line[i] = c[i];
  lcd.setCursor(0, row);
  lcd.print(line);
}

// ------------------------------------------------------------------------------------
void findAndApplyOptimalSettings();
void setBlinds(float position);
void setLEDs(float level);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void handleIrInput();
void updateDisplay(String line0, String line1, String line2, String line3);
void runWashingMachineLogic();
void setWasherStatusColor(int red, int green, int blue);
void setWasherCycleColor(float washTime);
void blinkLED(int red, int green, int blue);

// ------------------------------------------------------------------------------------
// SETUP
void setup() {
  Serial.begin(9600);

  sensorIndoor.begin();
  blinds_servo.attach(pin_servo);
  pinMode(pin_led, OUTPUT);
  pinMode(pin_fan, OUTPUT);
  pinMode(pin_rgb_red, OUTPUT);
  pinMode(pin_rgb_green, OUTPUT);
  pinMode(pin_rgb_blue, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Smart Home System");
  lcd.setCursor(0, 1); lcd.print("Starting...");
  delay(500);

  IrReceiver.begin(pin_ir_receiver);

  setBlinds(b_applied);
  setLEDs(l_applied);
  analogWrite(pin_fan, 0);
  fan_pwm_last = 0;

  // --- Washer fuzzy init (simple) ---
  fuzzy_logic_input = new FuzzyInput(1);
  fuzzy->addFuzzyInput(fuzzy_logic_input);
  FuzzySet* level_low = new FuzzySet(0, 0, 0, 5);
  fuzzy_logic_input->addFuzzySet(level_low);
  FuzzySet* level_medium = new FuzzySet(2.5, 5, 5, 7.5);
  fuzzy_logic_input->addFuzzySet(level_medium);
  FuzzySet* level_high = new FuzzySet(5, 10, 10, 10);
  fuzzy_logic_input->addFuzzySet(level_high);

  washingCycle = new FuzzyOutput(1);
  fuzzy->addFuzzyOutput(washingCycle);
  FuzzySet* cycle_delicate = new FuzzySet(0, 0, 20, 40);
  washingCycle->addFuzzySet(cycle_delicate);
  FuzzySet* cycle_light = new FuzzySet(20, 40, 40, 60);
  washingCycle->addFuzzySet(cycle_light);
  FuzzySet* cycle_normal = new FuzzySet(40, 60, 60, 80);
  washingCycle->addFuzzySet(cycle_normal);
  FuzzySet* cycle_strong = new FuzzySet(60, 80, 180, 180);
  washingCycle->addFuzzySet(cycle_strong);

  FuzzyRuleAntecedent* if_level_low = new FuzzyRuleAntecedent(); if_level_low->joinSingle(level_low);
  FuzzyRuleConsequent* then_cycle_delicate = new FuzzyRuleConsequent(); then_cycle_delicate->addOutput(cycle_delicate); then_cycle_delicate->addOutput(cycle_light);
  fuzzy->addFuzzyRule(new FuzzyRule(1, if_level_low, then_cycle_delicate));

  FuzzyRuleAntecedent* if_level_medium = new FuzzyRuleAntecedent(); if_level_medium->joinSingle(level_medium);
  FuzzyRuleConsequent* then_cycle_normal = new FuzzyRuleConsequent(); then_cycle_normal->addOutput(cycle_normal);
  fuzzy->addFuzzyRule(new FuzzyRule(2, if_level_medium, then_cycle_normal));

  FuzzyRuleAntecedent* if_level_high = new FuzzyRuleAntecedent(); if_level_high->joinSingle(level_high);
  FuzzyRuleConsequent* then_cycle_strong = new FuzzyRuleConsequent(); then_cycle_strong->addOutput(cycle_strong);
  fuzzy->addFuzzyRule(new FuzzyRule(3, if_level_high, then_cycle_strong));

  // Boot screen
  lcdPrint20S(0, "Tin:0.0 Tout:0.0");
  lcdPrint20S(1, "Tset:21.0 LDR:0");
  lcdPrint20S(2, "Lux:0  P:0  AC:-");   // P = total power (W)
  lcdPrint20S(3, "B:50% L:0% F:0%");

  findAndApplyOptimalSettings();
}

// -----------------------------------------------------------------------------------
// LOOP
void loop() {
  handleIrInput();

  // Fast UI (100ms): update InT/OutT & Light immediately
  if (current_mode == MODE_NORMAL_OPERATION && (millis() - lastSensorUiTime >= SENSOR_UI_INTERVAL)) {
    lastSensorUiTime = millis();
    light_raw_inst = analogRead(pin_photosensor);
    int   potValue  = analogRead(pin_pot_tout);
    float T_out_raw = mapfloat(potValue, 0, 1023, -10.0, 40.0);
    float T_in_now  = sensorIndoor.readTemperature();
    static float T_in_display = 0;
    if (!isnan(T_in_now)) T_in_display = T_in_now;

    lcdPrint20S(0, "Tin:" + String(T_in_display,1) + " Tout:" + String(T_out_raw,1));
    lcdPrint20S(1, "Tset:" + String(T_in_set,1) + " LDR:" + String(light_raw_inst));
  }

  // Optimizer cadence
  if (current_mode == MODE_NORMAL_OPERATION) {
    if (millis() - lastRefreshTime >= OPTIMIZATION_INTERVAL) {
      lastRefreshTime = millis();
      findAndApplyOptimalSettings();
    }
  }
}

// -----------------------------------------------------------------------------------
// IR function
void handleIrInput() {
  if (!IrReceiver.decode()) return;
  unsigned long code = IrReceiver.decodedIRData.decodedRawData;
  if (code == 0xFFFFFFFF) { IrReceiver.resume(); return; }

  if (current_mode == MODE_NORMAL_OPERATION) {
    if (code == REMOTE_CODE_STAR) {
      current_mode = MODE_SET_TEMPERATURE;
      user_input_buffer = 0;
      updateDisplay("Enter Temp (C):", String(user_input_buffer), "", "OK to confirm");
      IrReceiver.resume(); return;
    }
    if (code == REMOTE_CODE_HASH) {
      current_mode = MODE_SET_WASHER_TIME;
      user_input_buffer = 0;
      updateDisplay("Enter Time (24H):", String(user_input_buffer), "Format: HHMM", "OK to confirm");
      IrReceiver.resume(); return;
    }
  }

  int num = -1;
  if (code == REMOTE_CODE_0) num = 0; if (code == REMOTE_CODE_1) num = 1;
  if (code == REMOTE_CODE_2) num = 2; if (code == REMOTE_CODE_3) num = 3;
  if (code == REMOTE_CODE_4) num = 4; if (code == REMOTE_CODE_5) num = 5;
  if (code == REMOTE_CODE_6) num = 6; if (code == REMOTE_CODE_7) num = 7;
  if (code == REMOTE_CODE_8) num = 8; if (code == REMOTE_CODE_9) num = 9;

  switch (current_mode) {
    case MODE_SET_TEMPERATURE:
    case MODE_SET_WASHER_TIME:
      if (num != -1) {
        user_input_buffer = (user_input_buffer * 10) + num;
        if (current_mode == MODE_SET_TEMPERATURE)
          updateDisplay("Enter Temp (C):", String(user_input_buffer), "", "OK to confirm");
        else
          updateDisplay("Enter Time (24H):", String(user_input_buffer), "Format: HHMM", "OK to confirm");
      }
      if (code == REMOTE_CODE_OK) {
        if (current_mode == MODE_SET_TEMPERATURE) {
          T_in_set = user_input_buffer;
          current_mode = MODE_NORMAL_OPERATION;
          updateDisplay("Temp set to:", String(T_in_set) + "C", "Returning...", "");
          delay(250);
          findAndApplyOptimalSettings();
        } else {
          if (user_input_buffer > 2359 || (user_input_buffer % 100) > 59) {
            updateDisplay("Error: Invalid Time", "", "Press # to restart", "");
            delay(500);
            current_mode = MODE_NORMAL_OPERATION;
          } else {
            washer_input_time = user_input_buffer;
            current_mode = MODE_SET_WASHER_SOIL;
            updateDisplay("Select Soil Level:", "1)Low 2)Med 3)High", "", "");
          }
        }
      }
      break;

    case MODE_SET_WASHER_SOIL:
      if (num >= 1 && num <= 3) {
        washer_input_soil = num;
        current_mode = MODE_SET_WASHER_FABRIC;
        updateDisplay("Select Fabric Type:", "1)VSoft 2)Soft", "3)Normal 4)Hard", "");
      } else if (num != -1 || code == REMOTE_CODE_OK) {
        updateDisplay("Error: Invalid Input", "Press 1..3", "Press # to restart", "");
        delay(500);
        current_mode = MODE_NORMAL_OPERATION;
      }
      break;

    case MODE_SET_WASHER_FABRIC:
      if (num >= 1 && num <= 4) {
        washer_input_fabric = num;
        current_mode = MODE_SET_WASHER_LOAD;
        updateDisplay("Select Load Size:", "1)Small 2)Med 3)Large", "", "");
      } else if (num != -1 || code == REMOTE_CODE_OK) {
        updateDisplay("Error: Invalid Input", "Press 1..4", "Press # to restart", "");
        delay(500);
        current_mode = MODE_NORMAL_OPERATION;
      }
      break;

    case MODE_SET_WASHER_LOAD:
      if (num >= 1 && num <= 3) {
        washer_input_load = num;
        runWashingMachineLogic();
        current_mode = MODE_NORMAL_OPERATION;
        delay(500);
        findAndApplyOptimalSettings();
      } else if (num != -1 || code == REMOTE_CODE_OK) {
        updateDisplay("Error: Invalid Input", "Press 1..3", "Press # to restart", "");
        delay(500);
        current_mode = MODE_NORMAL_OPERATION;
      }
      break;

    default: break;
  }

  IrReceiver.resume();
}

// ----------------------------------------------------------------------------
// Washing Machine Function
void updateDisplay(String line0, String line1, String line2, String line3) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line0);
  lcd.setCursor(0, 1); lcd.print(line1);
  lcd.setCursor(0, 2); lcd.print(line2);
  lcd.setCursor(0, 3); lcd.print(line3);
}

void setWasherStatusColor(int red, int green, int blue) {
  analogWrite(pin_rgb_red, red);
  analogWrite(pin_rgb_green, green);
  analogWrite(pin_rgb_blue, blue);
}

void setWasherCycleColor(float washTime) {
  if (washTime <= 20) setWasherStatusColor(0, 0, 255);
  else if (washTime <= 40) setWasherStatusColor(0, 255, 0);
  else if (washTime <= 60) setWasherStatusColor(255, 255, 0);
  else if (washTime <= 90) setWasherStatusColor(255, 165, 0);
  else setWasherStatusColor(255, 0, 0);
}

void blinkLED(int red, int green, int blue) {
  for (int i = 0; i < 3; i++) {
    setWasherStatusColor(red, green, blue); delay(200);
    setWasherStatusColor(0, 0, 0);          delay(150);
  }
}

void runWashingMachineLogic() {
  Serial.println("LOGIC: Running Blended Fuzzy Washing Machine Controller...");
  int softness_level = 0;
  if (washer_input_fabric >= 4) {
    softness_level = 2;
  } else if (washer_input_fabric == 3) {
    softness_level = 1;
  } else {
    softness_level = 0;
  }
  int quantity_level = washer_input_load - 1;
  int dirtiness_level = washer_input_soil - 1;
  float average_level = (softness_level + quantity_level + dirtiness_level) / 3.0;
  float fuzzy_input_value = mapfloat(average_level, 0.0, 2.0, 0.0, 10.0);
  fuzzy->setInput(1, fuzzy_input_value);
  fuzzy->fuzzify();
  float calculatedTime = fuzzy->defuzzify(1);
  String cycleName = "Unknown";
  if (calculatedTime <= 35) cycleName = "Delicate";
  else if (calculatedTime <= 75) cycleName = "Light";
  else if (calculatedTime <= 115) cycleName = "Normal";
  else cycleName = "Strong";
  Serial.print("BLENDED LOGIC: Average level is "); Serial.print(average_level);
  Serial.print(". Calculated wash time: "); Serial.println(calculatedTime);
  int currentHour = washer_input_time / 100;
  int currentMinute = washer_input_time % 100;
  int currentTimeInMinutes = (currentHour * 60) + currentMinute;
  String startTimeMessage = "";
  String tmw = "";
  const int MORNING_START = 600; const int MORNING_END = 780;
  const int EVENING_START = 1020; const int EVENING_END = 1200;
  if ((currentTimeInMinutes >= MORNING_START && (currentTimeInMinutes + calculatedTime) <= MORNING_END) ||
      (currentTimeInMinutes >= EVENING_START && (currentTimeInMinutes + calculatedTime) <= EVENING_END)) {
    startTimeMessage = "Starting now";
    tmw = "";
  } else if (currentTimeInMinutes < MORNING_START && (MORNING_START + calculatedTime) <= MORNING_END) {
    startTimeMessage = "Starting at 10:00";
    tmw = "";
  } else if (currentTimeInMinutes < EVENING_START && (EVENING_START + calculatedTime) <= EVENING_END) {
    startTimeMessage = "Starting at 17:00";
    tmw = "";
  } else {
    startTimeMessage = "Starting at 10:00";
    tmw = "tomorrow";
  }
  updateDisplay("Cycle: " + cycleName, "Wash Time: " + String(calculatedTime, 0) + " min", startTimeMessage, tmw);
  if (startTimeMessage == "Starting now") {
    setWasherCycleColor(calculatedTime);
  } else {
    blinkLED(255, 255, 0);
  }
}

// ----------------------------------------------------------------------------
// Climate optimizer - minimizing power with hard lux constraint
void findAndApplyOptimalSettings() {
  // Sensors
  float T_in = sensorIndoor.readTemperature();
  if (isnan(T_in)) { Serial.println("SENSOR_ERROR"); return; }
  int   potValue  = analogRead(pin_pot_tout);
  float T_out_raw = mapfloat(potValue, 0, 1023, -10.0, 40.0);
  light_raw_inst  = analogRead(pin_photosensor);

  // Sunlight → room lux scale
  float sun_lux_source = mapfloat((float)light_raw_inst, 0, 1023, 0, 30000.0f);
  if (sun_lux_source < 0) sun_lux_source = 0;

  // Lux requirement (relax when hot)
  float temp_err     = T_in - T_in_set;
  float temp_err_eff = (fabsf(temp_err) <= TEMP_DEADBAND) ? 0.0f : temp_err;
  float lux_min = (temp_err_eff > 0.0f) ? max(250.0f, lux_target - 50.0f) : lux_target;

  // Grid search at 1% resolution
  float best_cost = FLT_MAX, best_Q_total = 0, best_total_lux = 0, best_P_tot = 0;
  float b_best_raw = b_applied, l_best_raw = l_applied;

  for (int bi = 0; bi <= 100; ++bi) {
    float b = bi * 0.01f;
    float t_b = t_closed + (1.0f - t_closed) * b;                  // optical transmission
    float E_sun_room = sun_lux_source * t_b * (A_win / A_total) * 0.45 * tau_glass/0.76;

    for (int li = 0; li <= 100; ++li) {
      float l = li * 0.01f;
      float l_eff = l;

      // Night + closed-blinds minima
      if (light_raw_inst < NIGHT_RAW_THRESH && l_eff < LED_MIN_NIGHT) l_eff = LED_MIN_NIGHT;
      if (b <= B_CLOSED_MAX && l_eff < LED_MIN_CLOSED) l_eff = LED_MIN_CLOSED;

      // Lux from LEDs (perceptual) + sun
      float E_led = E_led_max * l_eff;
      float total_lux = E_sun_room + E_led;

      // Heat & power
      float Q_sun = sun_lux_source* A_win * t_b * shgc * le_sun;
      float blind_factor = 0.3f + (0.7f * b); // open conducts more
      float U_win_eff = U_win * blind_factor;
      float Q_cond = ((U_walls * A_walls) + (U_win_eff * A_win)) * (T_out_raw - T_in);
      float P_led = P_led_max * l_eff;
      float Q_led = P_led * led_heat; 
      float Q_total = Q_sun + Q_cond + Q_led;

      // AC electrical power proxy (only if heat gain and we're hot)
      float P_ac = (Q_total > 0 && T_in > T_in_set) ? (Q_total / COP) : 0.0f;
      float P_tot = P_led + P_ac + P_fan_max;

      // HARD lux requirement
      float cost = P_tot + ((total_lux + LUX_DEADBAND < lux_min) ? LUX_HARD_PENALTY : 0.0f);

      if (cost < best_cost) {
        best_cost = cost;
        best_Q_total = Q_total;
        best_total_lux = total_lux;
        best_P_tot = P_tot;
        b_best_raw = b;
        l_best_raw = l_eff;
      }
    }
  }

  // Night: force fully closed blinds and ≥90% LED
  if (light_raw_inst < NIGHT_RAW_THRESH) {
    b_best_raw = 0.0f;
    if (l_best_raw < LED_NIGHT_TARGET) l_best_raw = LED_NIGHT_TARGET;
  }
  // Closed blinds enforce LED min
  if (b_best_raw <= B_CLOSED_MAX && l_best_raw < LED_MIN_CLOSED) l_best_raw = LED_MIN_CLOSED;

  // Fan target (linear on temperature error)
  float fan_target = 0.0f;
  float errC = T_in - T_in_set;                      // °C above setpoint
  if (errC > 0.0f) {
    fan_target = clamp(errC / FAN_ERR_FULL, 0.0f, 1.0f);
  }

  int fan_pwm_val = (int)roundf(fan_target * 255.0f);
  analogWrite(pin_fan, fan_pwm_val);
  fan_pwm_last = fan_pwm_val;

  // Apply immediately
  setBlinds(b_best_raw);
  setLEDs(l_best_raw);
  b_applied = b_best_raw;
  l_applied = l_best_raw;

  // Display power (recompute P_tot with applied state for LCD)
  float led_perceived_disp = pow(l_applied, LED_LUX_GAMMA);
  float E_led_disp = E_led_max * LED_LUX_EFF * led_perceived_disp;
  float t_b_disp = t_closed + (1.0f - t_closed) * b_applied;
  float E_sun_room_disp = sun_lux_source * t_b_disp * (A_win / A_total) * 0.45 * tau_glass/0.76;
  float total_lux_disp = E_sun_room_disp + E_led_disp;
  float Q_sun_disp =  sun_lux_source* A_win * t_b_disp * shgc * le_sun;
  float U_win_eff_disp = U_win * (0.3f + 0.7f * b_applied);
  float Q_cond_disp = ((U_walls * A_walls) + (U_win_eff_disp * A_win)) * (T_out_raw - T_in);
  float P_led_disp = P_led_max * l_applied;
  float Q_led_disp = P_led_disp * led_heat; 
  float Q_total_disp = Q_sun_disp + Q_cond_disp + Q_led_disp;
  float P_ac_disp = (Q_total_disp > 0 && T_in > T_in_set) ? (Q_total_disp / COP) : 0.0f;
  float P_fan_disp = P_fan_max * ((float)fan_pwm_last / 255.0f);
  float P_tot_disp = P_led_disp + P_ac_disp + P_fan_disp;
  String ac_status = (P_ac_disp > 0.0f) ? "ON" : "OFF";

// ----------------------------------------------------------------------------
// LCD display
  int fan_percent = map(fan_pwm_val, 0, 255, 0, 100);
  lcdPrint20S(0, "Tin:" + String(T_in,1) + " Tout:" + String(T_out_raw,1));
  lcdPrint20S(1, "Tset:" + String(T_in_set,1) + " LDR:" + String(light_raw_inst));
  lcdPrint20S(2, "Lux:" + String((int)roundf(total_lux_disp)) + " P:" + String((int)roundf(P_tot_disp)) + " AC:" + (ac_status));
  lcdPrint20S(3, "B:" + String((int)roundf(b_applied*100)) +
                 "% L:" + String((int)roundf(l_applied*100)) +
                 "% F:" + String(fan_percent) + "%");
                 
// ----------------------------------------------------------------------------
// Serial monitor output
  Serial.print(T_in_set,1);   Serial.print(",");
  Serial.print(T_out_raw,1); Serial.print(",");
  Serial.print(T_in,1);  Serial.print(",");
  Serial.print(sun_lux_source);  Serial.print(",");
  Serial.print(E_led_disp); Serial.print(",");
  Serial.print(E_sun_room_disp); Serial.print(",");
  Serial.print((int)roundf(total_lux_disp)); Serial.print(",");
  Serial.print(Q_led_disp, 1);Serial.print(",");
  Serial.print(Q_sun_disp, 1);Serial.print(",");
  Serial.print(Q_cond_disp, 1);Serial.print(",");
  Serial.print(Q_total_disp, 1);Serial.print(",");
  Serial.print(P_led_disp,2); Serial.print(",");
  Serial.print(P_ac_disp,2);  Serial.print(",");
  Serial.print(P_fan_disp,2);Serial.print(",");
  Serial.print(P_tot_disp,2);Serial.print(",");
  Serial.print(b_applied,2);  Serial.print(",");
  Serial.println(l_applied,2); 
}

// ----------------------------------------------------------------------------
// Hardware
// Blinds
void setBlinds(float position) {
  position = clamp(position, 0.0f, 1.0f);
  int fromDeg = BLINDS_INVERT ? BLINDS_MAX_DEG : BLINDS_MIN_DEG;
  int toDeg   = BLINDS_INVERT ? BLINDS_MIN_DEG : BLINDS_MAX_DEG;
  int angle = map((int)(position * 100), 0, 100, fromDeg, toDeg); // 1% steps
  blinds_servo.write(angle);
}

//LEDs
void setLEDs(float level) {
  level = clamp(level, 0.0f, 1.0f);
  int duty = map((int)(level * 100), 0, 100, 0, 255); // 1% steps
  analogWrite(pin_led, duty);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
