#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <LiquidCrystal_I2C.h>
#include <MD_MAX72xx.h>
#include <MD_Parola.h>
#include <Preferences.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

// ===== Hardware Setup =====
#define SDA_PIN 21
#define SCL_PIN 22
#define MAX_DATA_PIN 23
#define MAX_CS_PIN 5
#define MAX_CLK_PIN 18
#define SERVO_PIN 27
#define BUTTON_READ_PIN 15
#define BUTTON_GND_PIN 4
#define POT_PIN 34

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
MD_Parola matrix =
    MD_Parola(MD_MAX72XX::FC16_HW, MAX_DATA_PIN, MAX_CLK_PIN, MAX_CS_PIN, 4);
Servo myServo;
Preferences prefs;
BluetoothSerial SerialBT;

// ===== BLE OBD-II =====
// use the BLEAddress, serviceUUID, and ioUUID of your obd to ble adaptor.
static BLEAddress obdAddr("81:23:45:67:89:BA");
static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
static BLEUUID ioUUID("0000fff1-0000-1000-8000-00805f9b34fb");
BLEClient *bleClient;
BLERemoteCharacteristic *ioChar;

// Async Polling Variables
// Raw buffer for BLE callback (avoiding String in interrupt)
char rawResponse[128];
volatile int rawIdx = 0;
volatile bool responseReady = false;
bool obdConnected = false;

enum PollState { IDLE, SEND_CMD, WAIT_RESP };
PollState pollState = IDLE;
unsigned long cmdSentTime = 0;
int activePIDs[26];
int activePIDCount = 0;
int currentPollIdx = 0;

// ===== Vehicle Data =====
int engineRPM = 0;
int vehicleSpeed = 0;
int coolantTemp = -40;
int intakeTemp = -40;
int ambientTemp = -40;
int absLoad = 0;
int pedalPos = 0;
float controlVoltage = 0.0;
int timeSinceStart = 0;

// New OBD
int oilTemp = -40;
float lambdaB1S1 = 0.0;
float evapPurge = 0.0;
float fuelLevel = 0.0;
int baroPressure = 0;
float cmdEqRatio = 0.0;

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

const char *const dayNames[] = {"Sun", "Mon", "Tue", "Wed",
                                "Thu", "Fri", "Sat"};

void getIST(int &y, int &M, int &d, int &h, int &m, int &s) {
  m += 30;
  if (m >= 60) {
    m -= 60;
    h += 1;
  }
  h += 5;
  if (h >= 24) {
    h -= 24;
    d += 1;
    int daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))
      daysInMonth[2] = 29;
    if (d > daysInMonth[M]) {
      d = 1;
      M += 1;
      if (M > 12) {
        M = 1;
        y += 1;
      }
    }
  }
}

int getDayOfWeek(int y, int m, int d) {
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3)
    y -= 1;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

// ===== Wi-Fi & API Variables =====
String wifiSSID = "CDASTRAJAN";
String wifiPass = "vyshnavbs123";
bool wifiConnected = false;
bool internetConnected = false;
String localIPStr = "0.0.0.0";

// ===== Geolocation (Nominatim) =====
String postcodeStr = "No Fix";
String addressStr = "No Fix";
double lastNetLat = 0.0;
double lastNetLon = 0.0;
unsigned long lastNetRequestTime = 0;
bool netFetchPending = true;

// ===== Air Quality (AQICN) =====
int aqiValue = -1;
float pm25Value = -1.0;
float pm10Value = -1.0;
float coValue = -1.0;
float no2Value = -1.0;
float o3Value = -1.0;
int activeAqSubParam = 0; // 0: AQI, 1: PM2.5, 2: PM10, 3: CO, 4: NO2, 5: Ozone
const char *const aqSubParamNames[] = {"AQI", "PM2.5", "PM10",
                                       "CO",  "NO2",   "O3"};

// ===== Cricket Score (CricAPI) =====
int cricketTeam1Idx = 0;
int cricketTeam2Idx = 0;
const char *const popularTeams[] = {"ALL", "IND", "AUS", "ENG", "CSK", "RCB",
                                    "MI",  "GT",  "SRH", "RR",  "KKR", "PAK",
                                    "NZ",  "SA",  "WI",  "SL",  "BAN"};
const int numPopularTeams = 17;
int cricketPushTarget = 0; // 0: ALL, 1: Matrix, 2: OLED, 3: LCD, 4: NONE
String cricketScore1 = "No Live Matches";
String lastCricketScore1 = "";
String cricketScore2 = "No Live Matches";
String lastCricketScore2 = "";
unsigned long lastCricketFetchTime = 0;
bool cricketPushActive = false;
unsigned long cricketPushStartTime = 0;
String cricketPushMsg = "";

// ===== Distance Calculator =====
bool distCalcRunning = false;
double distCalcTotal = 0.0;        // in meters
double distCalcDisplacement = 0.0; // in meters
double distStartLat = 0.0;
double distStartLon = 0.0;
double distLastLat = 0.0;
double distLastLon = 0.0;
bool distStartPosAcquired = false;

// ===== Haversine Formula Helper =====
double calcDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * DEG_TO_RAD) *
                                                 cos(lat2 * DEG_TO_RAD) *
                                                 sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371000.0 * c; // returns distance in meters
}

// ===== Custom Lightweight JSON Parser =====
String findJsonValue(const String &json, const String &key) {
  int keyIdx = json.indexOf("\"" + key + "\":");
  if (keyIdx == -1) {
    keyIdx = json.indexOf("\"" + key + "\" :");
  }
  if (keyIdx == -1)
    return "";

  int startIdx = json.indexOf(":", keyIdx);
  if (startIdx == -1)
    return "";
  startIdx++; // move past ':'

  // Skip spaces
  while (startIdx < json.length() &&
         (json[startIdx] == ' ' || json[startIdx] == '\t')) {
    startIdx++;
  }

  if (startIdx >= json.length())
    return "";

  // If it's a string, find matching quote
  if (json[startIdx] == '"') {
    startIdx++;
    int endIdx = json.indexOf("\"", startIdx);
    if (endIdx == -1)
      return "";
    return json.substring(startIdx, endIdx);
  }

  // If it's a number, boolean, or null, read until comma, bracket, brace, or
  // space/newline
  int endIdx = startIdx;
  while (endIdx < json.length()) {
    char c = json[endIdx];
    if (c == ',' || c == '}' || c == ']' || c == '\r' || c == '\n' ||
        c == ' ') {
      break;
    }
    endIdx++;
  }
  return json.substring(startIdx, endIdx);
}

String findNestedJsonValue(const String &json, const String &parentKey,
                           const String &childKey) {
  int parentIdx = json.indexOf("\"" + parentKey + "\"");
  if (parentIdx == -1)
    return "";

  // Do not search for the closing bracket as it breaks on nested JSON structures.
  // Instead, just search forward from the parent key.
  String subJson = json.substring(parentIdx);
  return findJsonValue(subJson, childKey);
}

// ===== Parser for Cricket Score =====
void parseCricketScore(const String &json) {
  int dataIdx = json.indexOf("\"data\"");
  if (dataIdx == -1)
    return;

  int arrayStart = json.indexOf("[", dataIdx);
  if (arrayStart == -1)
    return;

  String targetTeam1 = String(popularTeams[cricketTeam1Idx]);
  if (targetTeam1 == "ALL")
    targetTeam1 = "";
  String targetTeam2 = String(popularTeams[cricketTeam2Idx]);
  if (targetTeam2 == "ALL")
    targetTeam2 = "";

  bool found1 = false;
  bool found2 = false;

  int searchPos = arrayStart;
  while (true) {
    int matchStart = json.indexOf("{", searchPos);
    if (matchStart == -1)
      break;

    int matchEnd = json.indexOf("},", matchStart);
    if (matchEnd == -1) {
      matchEnd = json.indexOf("}", matchStart);
    }
    if (matchEnd == -1)
      break;

    String matchStr = json.substring(matchStart, matchEnd + 1);
    searchPos = matchEnd + 1;

    String matchNameLower = matchStr;
    matchNameLower.toLowerCase();

    bool matchesTeam1 = false;
    bool matchesTeam2 = false;

    if (targetTeam1 == "") {
      if (!found1 && matchStr.indexOf("\"score\"") != -1 &&
          matchStr.indexOf("\"r\"") != -1)
        matchesTeam1 = true;
    } else {
      String t1Lower = targetTeam1;
      t1Lower.toLowerCase();
      if (!found1 && matchNameLower.indexOf(t1Lower) != -1)
        matchesTeam1 = true;
    }

    if (targetTeam2 == "") {
      if (!found2 && matchStr.indexOf("\"score\"") != -1 &&
          matchStr.indexOf("\"r\"") != -1)
        matchesTeam2 = true;
    } else {
      String t2Lower = targetTeam2;
      t2Lower.toLowerCase();
      if (!found2 && matchNameLower.indexOf(t2Lower) != -1)
        matchesTeam2 = true;
    }

    if (matchesTeam1 || matchesTeam2) {
      String name = findJsonValue(matchStr, "name");
      String status = findJsonValue(matchStr, "status");

      int scoreArrayIdx = matchStr.indexOf("\"score\"");
      String scoreStr1 = "";
      String scoreStr2 = "";

      if (scoreArrayIdx != -1) {
        int score1Start = matchStr.indexOf("{", scoreArrayIdx);
        if (score1Start != -1 &&
            score1Start < matchStr.indexOf("]", scoreArrayIdx)) {
          int score1End = matchStr.indexOf("}", score1Start);
          if (score1End != -1) {
            String s1 = matchStr.substring(score1Start, score1End + 1);
            String r1 = findJsonValue(s1, "r");
            String w1 = findJsonValue(s1, "w");
            String o1 = findJsonValue(s1, "o");
            String inn1 = findJsonValue(s1, "inning");

            String t1 = inn1.substring(0, 3);
            t1.toUpperCase();
            scoreStr1 = t1 + " " + r1 + "/" + w1 + " (" + o1 + "ov)";

            int score2Start = matchStr.indexOf("{", score1End + 1);
            if (score2Start != -1 &&
                score2Start < matchStr.indexOf("]", scoreArrayIdx)) {
              int score2End = matchStr.indexOf("}", score2Start);
              if (score2End != -1) {
                String s2 = matchStr.substring(score2Start, score2End + 1);
                String r2 = findJsonValue(s2, "r");
                String w2 = findJsonValue(s2, "w");
                String o2 = findJsonValue(s2, "o");
                String inn2 = findJsonValue(s2, "inning");

                String t2 = inn2.substring(0, 3);
                t2.toUpperCase();
                scoreStr2 = t2 + " " + r2 + "/" + w2 + " (" + o2 + "ov)";
              }
            }
          }
        }
      }

      String result = "";
      if (scoreStr1 != "") {
        result += scoreStr1;
        if (scoreStr2 != "") {
          result += " v " + scoreStr2;
        }
      } else {
        result = name + " - " + status;
      }

      if (matchesTeam1) {
        cricketScore1 = result;
        found1 = true;
      }
      if (matchesTeam2) {
        cricketScore2 = result;
        found2 = true;
      }
    }
  }

  if (!found1)
    cricketScore1 = "No Live Matches";
  if (!found2)
    cricketScore2 = "No Live Matches";

  if (cricketPushTarget != 4) {
    bool pushTriggered = false;
    String pushMsg = "";
    if (lastCricketScore1 != "" && lastCricketScore1 != cricketScore1) {
      pushTriggered = true;
      pushMsg = "T1: " + cricketScore1;
    } else if (lastCricketScore2 != "" && lastCricketScore2 != cricketScore2) {
      pushTriggered = true;
      pushMsg = "T2: " + cricketScore2;
    }
    if (pushTriggered) {
      cricketPushActive = true;
      cricketPushStartTime = millis();
      cricketPushMsg = pushMsg;
    }
  }
  lastCricketScore1 = cricketScore1;
  lastCricketScore2 = cricketScore2;
}

// ===== Display Value Getter Helpers =====
String getDistanceStr() {
  char buf[32];
  double km = distCalcTotal / 1000.0;
  sprintf(buf, "D:%.2fkm", km);
  return String(buf);
}

String getDisplacementStr() {
  char buf[32];
  double dispKm = distCalcDisplacement / 1000.0;
  sprintf(buf, "Dsp:%.2fkm", dispKm);
  return String(buf);
}

String getAirQualityStr() {
  if (aqiValue == -1)
    return "Loading AQI...";
  switch (activeAqSubParam) {
  case 0:
    return "AQI: " + String(aqiValue);
  case 1:
    return "PM2.5: " + (pm25Value >= 0 ? String(pm25Value, 1) : "N/A");
  case 2:
    return "PM10: " + (pm10Value >= 0 ? String(pm10Value, 1) : "N/A");
  case 3:
    return "CO: " + (coValue >= 0 ? String(coValue, 1) : "N/A");
  case 4:
    return "NO2: " + (no2Value >= 0 ? String(no2Value, 1) : "N/A");
  case 5:
    return "O3: " + (o3Value >= 0 ? String(o3Value, 1) : "N/A");
  default:
    return "AQI: " + String(aqiValue);
  }
}

String getAddressStr() { return addressStr; }

String getPincodeStr() { return postcodeStr; }

void checkLocationChanges() {
  if (!gps.location.isValid())
    return;
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  if (lastNetLat == 0.0 && lastNetLon == 0.0) {
    lastNetLat = lat;
    lastNetLon = lon;
    netFetchPending = true;
  } else {
    double dist = calcDistance(lat, lon, lastNetLat, lastNetLon);
    if (dist > 200.0) {
      lastNetLat = lat;
      lastNetLon = lon;
      netFetchPending = true;
    }
  }
}

void updateDistanceCalculator() {
  if (!distCalcRunning)
    return;

  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();

    if (!distStartPosAcquired) {
      distStartLat = lat;
      distStartLon = lon;
      distLastLat = lat;
      distLastLon = lon;
      distStartPosAcquired = true;
      distCalcTotal = 0.0;
      distCalcDisplacement = 0.0;
    } else {
      double incDist = calcDistance(distLastLat, distLastLon, lat, lon);
      // Avoid small GPS jitter noise when stationary
      if (incDist > 1.5 && gps.speed.kmph() > 1.0) {
        distCalcTotal += incDist;
        distLastLat = lat;
        distLastLon = lon;
      }
      distCalcDisplacement = calcDistance(distStartLat, distStartLon, lat, lon);
    }
  }
}

const char *const paramNames[] = {
    "None",        "RPM",      "Speed",        "Coolant",  "Intake",
    "Ambient",     "Load",     "Pedal",        "Battery",  "Run Time",
    "OilT",        "Lambda",   "Purge",        "FuelLvl",  "Baro",
    "CmdEq",       "GPSSpd",   "Heading",      "Date",     "Time",
    "Time(s)",     "Alt",      "Lat",          "Lon",      "Sats",
    "GPSFix",      "Week",     "FullDate",     "Cricket1", "Cricket2",
    "Air Quality", "Distance", "Displacement", "Address",  "Pincode"};

String getParamName(int idx) {
  if (idx == 28)
    return String(popularTeams[cricketTeam1Idx]);
  if (idx == 29)
    return String(popularTeams[cricketTeam2Idx]);
  return String(paramNames[idx]);
}

String getParamStr(int paramIdx) {
  switch (paramIdx) {
  case 1:
    return String(engineRPM);
  case 2:
    return String(vehicleSpeed);
  case 3:
    return String(coolantTemp) + "C";
  case 4:
    return String(intakeTemp) + "C";
  case 5:
    return String(ambientTemp) + "C";
  case 6:
    return String(absLoad) + "%";
  case 7:
    return String(pedalPos) + "%";
  case 8:
    return String(controlVoltage, 1) + "V";
  case 9: {
    int m = timeSinceStart / 60;
    int s = timeSinceStart % 60;
    return String(m) + "m " + String(s) + "s";
  }
  case 10:
    return String(oilTemp) + "C";
  case 11:
    return String(lambdaB1S1, 2);
  case 12:
    return String(evapPurge, 1) + "%";
  case 13:
    return String(fuelLevel, 1) + "%";
  case 14:
    return String(baroPressure) + "kPa";
  case 15:
    return String(cmdEqRatio, 2);
  case 16:
    return String(gps.speed.kmph(), 1);
  case 17:
    return String(gps.course.deg(), 1);
  case 18: {
    int y = gps.date.year(), M = gps.date.month(), d = gps.date.day();
    int h = gps.time.hour(), m = gps.time.minute(), s = gps.time.second();
    getIST(y, M, d, h, m, s);
    char buf[12];
    sprintf(buf, "%02d/%02d/%04d", d, M, y);
    return String(buf);
  }
  case 19: {
    int y = gps.date.year(), M = gps.date.month(), d = gps.date.day();
    int h = gps.time.hour(), m = gps.time.minute(), s = gps.time.second();
    getIST(y, M, d, h, m, s);
    h %= 12;
    if (h == 0)
      h = 12;
    char buf[10];
    sprintf(buf, "%02d:%02d", h, m);
    return String(buf);
  }
  case 20: {
    int y = gps.date.year(), M = gps.date.month(), d = gps.date.day();
    int h = gps.time.hour(), m = gps.time.minute(), s = gps.time.second();
    getIST(y, M, d, h, m, s);
    h %= 12;
    if (h == 0)
      h = 12;
    char buf[10];
    sprintf(buf, "%02d:%02d:%02d", h, m, s);
    return String(buf);
  }
  case 21:
    return String(gps.altitude.meters(), 1) + "m";
  case 22:
    return String(gps.location.lat(), 4);
  case 23:
    return String(gps.location.lng(), 4);
  case 24:
    return String(gps.satellites.value());
  case 25:
    return gps.location.isValid() ? "Fixed" : "No Fix";
  case 26: {
    int y = gps.date.year(), M = gps.date.month(), d = gps.date.day();
    int h = gps.time.hour(), m = gps.time.minute(), s = gps.time.second();
    getIST(y, M, d, h, m, s);
    return String(dayNames[getDayOfWeek(y, M, d)]);
  }
  case 27: {
    int y = gps.date.year(), M = gps.date.month(), d = gps.date.day();
    int h = gps.time.hour(), m = gps.time.minute(), s = gps.time.second();
    getIST(y, M, d, h, m, s);
    String dayOfWeek = String(dayNames[getDayOfWeek(y, M, d)]);
    char buf[32];
    sprintf(buf, "%04d-%02d %s %02d", y, M, dayOfWeek.c_str(), d);
    return String(buf);
  }
  case 28:
    return cricketScore1;
  case 29:
    return cricketScore2;
  case 30:
    return getAirQualityStr();
  case 31:
    return getDistanceStr();
  case 32:
    return getDisplacementStr();
  case 33:
    return getAddressStr();
  case 34:
    return getPincodeStr();
  default:
    return "";
  }
}

String getCmdForTarget(int target) {
  switch (target) {
  case 1:
    return "010C";
  case 2:
    return "010D";
  case 3:
    return "0105";
  case 4:
    return "010F";
  case 5:
    return "0146";
  case 6:
    return "0104";
  case 7:
    return "014A";
  case 8:
    return "0142";
  case 9:
    return "011F";
  case 10:
    return "015C";
  case 11:
    return "0124";
  case 12:
    return "012E";
  case 13:
    return "012F";
  case 14:
    return "0133";
  case 15:
    return "0144";
  default:
    return "";
  }
}

// ===== UI State =====
enum UIMode { UI_NORMAL, UI_SETTINGS, UI_DISP_MODE };
enum MenuLevel { LEVEL_MAIN, LEVEL_SUB };
UIMode uiMode = UI_NORMAL;
MenuLevel menuLevel = LEVEL_MAIN;

int mainMenuIdx = 0;
int subMenuIdx = 0;
int matrixTarget = 1;
int lcdMode = 0;
int lcdRow1Target = 8;
int lcdRow2Target = 6;
int servoTarget = 1;
bool matrixPower = true;
bool lcdPower = true;
bool servoPower = true;
int matrixBrightness = 5;
int oledTarget = 1;
bool oledParamPower = false;
bool oledSleepEn = true;
int oledSleepTime = 10; // seconds
bool lcdSleepEn = true;
int lcdSleepTime = 10;
bool matrixSleepEn = true;
int matrixSleepTime = 10;
int matrixMode = 0; // 0=Normal, 1=Strobe
bool servoTestMode = false;
bool settingsDirty = false;
unsigned long lastSaveTime = 0;

// Helper to format seconds to string
String formatSecs(int s) {
  if (s < 60)
    return String(s) + "s";
  int m = s / 60;
  int rs = s % 60;
  if (rs == 0)
    return String(m) + "m";
  return String(m) + "m" + String(rs) + "s";
}

// Display Mode Vars
String btText = "AETHERON";
int dispBrightness = 5;
int dispSpeed = 30;
int dispDirection = 0; // 0=Left, 1=Right
bool dispScrollEn = true;
bool btInitialized = false;

bool globalPower = true;
bool sleepEnabled = true;

int settingsMenuIdx = 0;
bool isEditingSetting = false;
String lastMatrixStr = ""; // caching to prevent flicker

// Input / Sleep State
bool lastButtonState = HIGH;
bool buttonReading = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long buttonPressTime = 0;
int lastRawPot = -1;
unsigned long lastActivityTime = 0;
bool isOledAsleep = false;
bool isLcdAsleep = false;
bool isMatrixAsleep = false;

// ======================= MEMORY & STATE =======================
void loadSettings() {
  prefs.begin("obd", true);
  matrixTarget = prefs.getInt("m_tgt", 1);
  lcdMode = prefs.getInt("l_mod", 0);
  lcdRow1Target = prefs.getInt("l_r1", 8);
  lcdRow2Target = prefs.getInt("l_r2", 6);
  servoTarget = prefs.getInt("s_tgt", 1);
  matrixPower = prefs.getBool("m_pwr", true);
  lcdPower = prefs.getBool("l_pwr", true);
  servoPower = prefs.getBool("s_pwr", true);
  matrixBrightness = prefs.getInt("brt", 5);
  oledTarget = prefs.getInt("o_tgt", 1);
  oledParamPower = prefs.getBool("o_pwr", false);
  oledSleepEn = prefs.getBool("o_slp_e", true);
  oledSleepTime = constrain(prefs.getInt("o_slp_t", 10), 10, 600);
  lcdSleepEn = prefs.getBool("l_slp_e", true);
  lcdSleepTime = constrain(prefs.getInt("l_slp_t", 10), 10, 600);
  matrixSleepEn = prefs.getBool("m_slp_e", true);
  matrixSleepTime = constrain(prefs.getInt("m_slp_t", 10), 10, 600);
  matrixMode = prefs.getInt("m_mode", 0);
  dispBrightness = prefs.getInt("d_brt", 5);
  dispSpeed = prefs.getInt("d_spd", 30);
  dispDirection = prefs.getInt("d_dir", 0);
  dispScrollEn = prefs.getBool("d_scr", true);
  btText = prefs.getString("bt_txt", "AETHERON");
  cricketTeam1Idx = prefs.getInt("c_t1", 0);
  cricketTeam2Idx = prefs.getInt("c_t2", 0);
  cricketPushTarget = prefs.getInt("c_push", 0);
  prefs.end();
}

void saveSettings() {
  prefs.begin("obd", false);
  prefs.putInt("m_tgt", matrixTarget);
  prefs.putInt("l_mod", lcdMode);
  prefs.putInt("l_r1", lcdRow1Target);
  prefs.putInt("l_r2", lcdRow2Target);
  prefs.putInt("s_tgt", servoTarget);
  prefs.putBool("m_pwr", matrixPower);
  prefs.putBool("l_pwr", lcdPower);
  prefs.putBool("s_pwr", servoPower);
  prefs.putInt("brt", matrixBrightness);
  prefs.putInt("o_tgt", oledTarget);
  prefs.putBool("o_pwr", oledParamPower);
  prefs.putBool("o_slp_e", oledSleepEn);
  prefs.putInt("o_slp_t", oledSleepTime);
  prefs.putBool("l_slp_e", lcdSleepEn);
  prefs.putInt("l_slp_t", lcdSleepTime);
  prefs.putBool("m_slp_e", matrixSleepEn);
  prefs.putInt("m_slp_t", matrixSleepTime);
  prefs.putInt("m_mode", matrixMode);
  prefs.putInt("d_brt", dispBrightness);
  prefs.putInt("d_spd", dispSpeed);
  prefs.putInt("d_dir", dispDirection);
  prefs.putBool("d_scr", dispScrollEn);
  prefs.putString("bt_txt", btText);
  prefs.putInt("c_t1", cricketTeam1Idx);
  prefs.putInt("c_t2", cricketTeam2Idx);
  prefs.putInt("c_push", cricketPushTarget);
  prefs.end();
}

void buildActivePIDList() {
  bool wants[28] = {false};
  if (matrixPower && matrixTarget > 0)
    wants[matrixTarget] = true;
  if (servoPower && servoTarget > 0)
    wants[servoTarget] = true;
  if (lcdPower && lcdMode == 0) {
    if (lcdRow1Target > 0)
      wants[lcdRow1Target] = true;
    if (lcdRow2Target > 0)
      wants[lcdRow2Target] = true;
  }
  if (oledParamPower && oledTarget > 0)
    wants[oledTarget] = true;

  activePIDCount = 0;
  for (int i = 1; i <= 27; i++) {
    if (wants[i]) {
      String cmd = getCmdForTarget(i);
      if (cmd != "") {
        activePIDs[activePIDCount++] = i;
      }
    }
  }
}

// ======================= BLE & OBD FUNCTIONS =======================
void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                    uint8_t *pData, size_t length, bool isNotify) {
  for (size_t i = 0; i < length; i++) {
    char c = (char)pData[i];
    if (c == '>') {
      responseReady = true;
    } else {
      if (rawIdx < 127) {
        rawResponse[rawIdx++] = c;
        rawResponse[rawIdx] = '\0';
      }
    }
  }
}

void parseOBDResponse(String cmd, String resp) {
  String pidStr = cmd.substring(2, 4);
  String expectedPrefix = "41" + pidStr;

  int startIdx = resp.indexOf(expectedPrefix);
  if (startIdx == -1)
    return;

  String dataPart = resp.substring(startIdx + 4);

  if (pidStr == "0C" && dataPart.length() >= 4) { // RPM
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    int B = strtol(dataPart.substring(2, 4).c_str(), NULL, 16);
    engineRPM = (A * 256 + int(B)) / 4;
  } else if (pidStr == "0D" && dataPart.length() >= 2) { // Speed
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    vehicleSpeed = A;
  } else if (pidStr == "05" && dataPart.length() >= 2) { // Coolant Temp
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    coolantTemp = A - 40;
  } else if (pidStr == "0F" && dataPart.length() >= 2) { // Intake Temp
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    intakeTemp = A - 40;
  } else if (pidStr == "46" && dataPart.length() >= 2) { // Ambient Temp
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    ambientTemp = A - 40;
  } else if (pidStr == "04" &&
             dataPart.length() >= 2) { // Calculated Engine Load
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    absLoad = A * 100 / 255;
  } else if (pidStr == "4A" && dataPart.length() >= 2) { // Pedal Pos E
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    int rawPedal = A * 100 / 255;
    pedalPos = map(rawPedal, 19, 99, 0, 100);
    pedalPos = constrain(pedalPos, 0, 100);
  } else if (pidStr == "42" && dataPart.length() >= 4) { // Control Voltage
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    int B = strtol(dataPart.substring(2, 4).c_str(), NULL, 16);
    controlVoltage = (A * 256.0 + B) / 1000.0;
  } else if (pidStr == "1F" && dataPart.length() >= 4) { // Time Since Start
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    int B = strtol(dataPart.substring(2, 4).c_str(), NULL, 16);
    timeSinceStart = A * 256 + int(B);
  } else if (pidStr == "5C" && dataPart.length() >= 2) { // Oil Temp
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    oilTemp = A - 40;
  } else if (pidStr == "24" &&
             dataPart.length() >= 4) { // Lambda B1S1 (Equivalence Ratio)
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    int B = strtol(dataPart.substring(2, 4).c_str(), NULL, 16);
    lambdaB1S1 = (A * 256.0 + B) / 32768.0;
  } else if (pidStr == "2E" &&
             dataPart.length() >= 2) { // Commanded Evaporative Purge
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    evapPurge = A * 100.0 / 255.0;
  } else if (pidStr == "2F" && dataPart.length() >= 2) { // Fuel Level Input
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    fuelLevel = A * 100.0 / 255.0;
  } else if (pidStr == "33" && dataPart.length() >= 2) { // Barometric Pressure
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    baroPressure = A;
  } else if (pidStr == "44" &&
             dataPart.length() >= 4) { // Commanded Equivalence Ratio
    int A = strtol(dataPart.substring(0, 2).c_str(), NULL, 16);
    int B = strtol(dataPart.substring(2, 4).c_str(), NULL, 16);
    cmdEqRatio = (A * 256.0 + B) / 32768.0;
  }
}

void processAsyncOBD() {
  if (!obdConnected || activePIDCount == 0)
    return;

  if (pollState == IDLE || pollState == SEND_CMD) {
    if (currentPollIdx >= activePIDCount) {
      currentPollIdx = 0;
      buildActivePIDList();
      if (activePIDCount == 0)
        return;
    }

    // Reset raw buffer safely
    rawIdx = 0;
    rawResponse[0] = '\0';
    responseReady = false;
    cmdSentTime = millis();

    String cmd = getCmdForTarget(activePIDs[currentPollIdx]);
    ioChar->writeValue(cmd + "\r", false);
    pollState = WAIT_RESP;
  } else if (pollState == WAIT_RESP) {
    if (responseReady) {
      String resp = String(rawResponse);
      resp.replace(" ", "");
      resp.replace("\r", "");
      resp.replace("\n", "");

      parseOBDResponse(getCmdForTarget(activePIDs[currentPollIdx]), resp);

      currentPollIdx++;
      pollState = SEND_CMD;
    } else if (millis() - cmdSentTime >
               150) { // Slightly more relaxed timeout for stability
      Serial.printf("OBD Timeout on PID %d\n", activePIDs[currentPollIdx]);
      currentPollIdx++;
      pollState = SEND_CMD;
    }
  }
}

// ======================= BUTTON & POT LOGIC =======================
void checkInput() {
  // --- Sleep Logic (Per Component) ---
  unsigned long inactive = millis() - lastActivityTime;
  isOledAsleep =
      (oledSleepEn && inactive > (unsigned long)oledSleepTime * 1000);
  isLcdAsleep = (lcdSleepEn && inactive > (unsigned long)lcdSleepTime * 1000);
  isMatrixAsleep =
      (matrixSleepEn && inactive > (unsigned long)matrixSleepTime * 1000);

  // --- Potentiometer Logic ---
  int rawPot = analogRead(POT_PIN);
  int potDiff = abs(rawPot - lastRawPot);
  bool isAnyAsleep = isOledAsleep || isLcdAsleep || isMatrixAsleep;
  int threshold =
      isAnyAsleep
          ? 200
          : 200; // 200 to wake, 50 to stay awake (robust against rail noise)

  if (potDiff > threshold) {
    lastActivityTime = millis();
    lastRawPot = rawPot;
    int safePot = constrain(rawPot, 0, 4000);

    if (uiMode == UI_SETTINGS) {
      if (menuLevel == LEVEL_MAIN) {
        mainMenuIdx = map(safePot, 0, 4000, 0,
                          8); // OLED, LCD, MATRIX, SERVO, BLE DISPLAY, CRICKET,
                              // DISTANCE, WIFI, EXIT
      } else {
        // Nested Selection
        if (isEditingSetting) {
          switch (mainMenuIdx) {
          case 0: // OLED
            switch (subMenuIdx) {
            case 0:
              oledTarget = map(safePot, 0, 4000, 0, 34);
              break;
            case 1:
              oledParamPower = map(safePot, 0, 4000, 0, 1);
              break;
            case 2:
              oledSleepEn = map(safePot, 0, 4000, 0, 1);
              break;
            case 3: { // Minutes
              int m = map(safePot, 0, 4000, 0, 10);
              int s = oledSleepTime % 60;
              oledSleepTime = constrain(m * 60 + s, 10, 600);
              break;
            }
            case 4: { // Seconds
              int m = oledSleepTime / 60;
              int s = map(safePot, 0, 4000, 0, 59);
              oledSleepTime = constrain(m * 60 + s, 10, 600);
              break;
            }
            }
            break;
          case 1: // LCD
            switch (subMenuIdx) {
            case 0:
              lcdMode = map(safePot, 0, 4000, 0, 1);
              break;
            case 1:
              lcdRow1Target = map(safePot, 0, 4000, 0, 34);
              break;
            case 2:
              lcdRow2Target = map(safePot, 0, 4000, 0, 34);
              break;
            case 3:
              lcdPower = map(safePot, 0, 4000, 0, 1);
              break;
            case 4:
              lcdSleepEn = map(safePot, 0, 4000, 0, 1);
              break;
            case 5: { // Minutes
              int m = map(safePot, 0, 4000, 0, 10);
              int s = lcdSleepTime % 60;
              lcdSleepTime = constrain(m * 60 + s, 10, 600);
              break;
            }
            case 6: { // Seconds
              int m = lcdSleepTime / 60;
              int s = map(safePot, 0, 4000, 0, 59);
              lcdSleepTime = constrain(m * 60 + s, 10, 600);
              break;
            }
            }
            break;
          case 2: // MATRIX
            switch (subMenuIdx) {
            case 0:
              matrixTarget = map(safePot, 0, 4000, 0, 34);
              break;
            case 1:
              matrixPower = map(safePot, 0, 4000, 0, 1);
              break;
            case 2:
              matrixMode = map(safePot, 0, 4000, 0, 1);
              break;
            case 3:
              matrixBrightness = map(safePot, 0, 4000, 0, 15);
              break;
            case 4:
              matrixSleepEn = map(safePot, 0, 4000, 0, 1);
              break;
            case 5: { // Minutes
              int m = map(safePot, 0, 4000, 0, 10);
              int s = matrixSleepTime % 60;
              matrixSleepTime = constrain(m * 60 + s, 10, 600);
              break;
            }
            case 6: { // Seconds
              int m = matrixSleepTime / 60;
              int s = map(safePot, 0, 4000, 0, 59);
              matrixSleepTime = constrain(m * 60 + s, 10, 600);
              break;
            }
            }
            break;
          case 3: // SERVO
            switch (subMenuIdx) {
            case 0:
              servoTarget = map(safePot, 0, 4000, 0, 34);
              break;
            case 1:
              servoPower = map(safePot, 0, 4000, 0, 1);
              break;
            case 2: /* handled by toggle in checkInput */
              break;
            }
            break;
          case 5: // Cricket
            switch (subMenuIdx) {
            case 0:
              cricketTeam1Idx = map(safePot, 0, 4000, 0, numPopularTeams - 1);
              break;
            case 1:
              cricketTeam2Idx = map(safePot, 0, 4000, 0, numPopularTeams - 1);
              break;
            case 2:
              cricketPushTarget = map(safePot, 0, 4000, 0, 4);
              break;
            }
            break;
          }
        } else {
          int maxSub = 0;
          if (mainMenuIdx == 0)
            maxSub = 5; // OLED (Tgt, Pwr, Slp, Min, Sec, BACK)
          else if (mainMenuIdx == 1)
            maxSub = 7; // LCD (Mod, L1, L2, Pwr, Slp, Min, Sec, BACK)
          else if (mainMenuIdx == 2)
            maxSub = 7; // Matrix (Tgt, Pwr, Mode, Brt, Slp, Min, Sec, BACK)
          else if (mainMenuIdx == 3)
            maxSub = 3; // Servo (Tgt, Pwr, Test, BACK)
          else if (mainMenuIdx == 5)
            maxSub = 3; // Cricket (T1, T2, Push, BACK)
          else if (mainMenuIdx == 6)
            maxSub = 2; // Distance (Start/Stop, Reset, BACK)
          else if (mainMenuIdx == 7)
            maxSub = 3; // WiFi (Status, SSID, IP, BACK)
          subMenuIdx = map(safePot, 0, 4000, 0, maxSub);
        }
      }
    } else if (uiMode == UI_DISP_MODE) {
      if (isEditingSetting) {
        int newVal = 0;
        bool changed = false;
        switch (subMenuIdx) {
        case 0:
          newVal = map(safePot, 0, 4000, 0, 15);
          if (newVal != dispBrightness) {
            dispBrightness = newVal;
            changed = true;
          }
          break;
        case 1:
          newVal = map(safePot, 0, 4000, 150, 5);
          if (newVal != dispSpeed) {
            dispSpeed = newVal;
            changed = true;
          }
          break;
        case 2:
          newVal = map(safePot, 0, 4000, 0, 1);
          if (newVal != dispDirection) {
            dispDirection = newVal;
            changed = true;
          }
          break;
        case 3:
          newVal = map(safePot, 0, 4000, 0, 1);
          if (newVal != (int)dispScrollEn) {
            dispScrollEn = (bool)newVal;
            changed = true;
          }
          break;
        }
        if (changed) {
          settingsDirty = true;
          if (subMenuIdx != 0)
            lastMatrixStr = "";
        }
      } else {
        subMenuIdx = map(safePot, 0, 4000, 0, 4);
      }
    } else if (uiMode == UI_NORMAL) {
      if ((matrixPower && matrixTarget == 30) ||
          (lcdPower && (lcdRow1Target == 30 || lcdRow2Target == 30)) ||
          (oledParamPower && oledTarget == 30)) {
        int newVal = map(safePot, 0, 4000, 0, 5);
        if (newVal != activeAqSubParam) {
          activeAqSubParam = newVal;
          lastMatrixStr = "";
        }
      } else {
        bool hasCricket = false;
        if (matrixPower && (matrixTarget == 28 || matrixTarget == 29))
          hasCricket = true;
        if (oledParamPower && (oledTarget == 28 || oledTarget == 29))
          hasCricket = true;
        if (lcdPower && (lcdRow1Target == 28 || lcdRow1Target == 29 ||
                         lcdRow2Target == 28 || lcdRow2Target == 29))
          hasCricket = true;

        if (hasCricket) {
          int newVal = map(safePot, 0, 4000, 28, 29);
          if (matrixPower && (matrixTarget == 28 || matrixTarget == 29) &&
              matrixTarget != newVal) {
            matrixTarget = newVal;
            lastMatrixStr = "";
          }
          if (oledParamPower && (oledTarget == 28 || oledTarget == 29) &&
              oledTarget != newVal) {
            oledTarget = newVal;
          }
          if (lcdPower && (lcdRow1Target == 28 || lcdRow1Target == 29) &&
              lcdRow1Target != newVal) {
            lcdRow1Target = newVal;
          }
          if (lcdPower && (lcdRow2Target == 28 || lcdRow2Target == 29) &&
              lcdRow2Target != newVal) {
            lcdRow2Target = newVal;
          }
        }
      }
    }
  }

  // --- Button Logic (Fully Debounced) ---
  bool reading = digitalRead(BUTTON_READ_PIN);

  if (reading != buttonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastButtonState) {
      lastButtonState = reading;

      if (lastButtonState == LOW) {
        // Just Pressed
        buttonPressTime = millis();
      } else {
        // Just Released
        unsigned long duration = millis() - buttonPressTime;

        // Wake up hook (swallows the press if asleep)
        if (isOledAsleep) {
          lastActivityTime = millis();
          buttonPressTime = 0;
          return;
        }

        lastActivityTime = millis();

        if (duration > 4000) {
          globalPower = !globalPower;
          uiMode = UI_NORMAL;
        } else if (duration > 2000) { // Long Press 2s to Exit
          if (uiMode == UI_SETTINGS) {
            if (menuLevel == LEVEL_SUB)
              menuLevel = LEVEL_MAIN;
            else
              uiMode = UI_NORMAL;
          } else if (uiMode == UI_DISP_MODE) {
            uiMode = UI_NORMAL;
          }
        } else if (duration > 50) { // Short Press
          if (!globalPower) {
            globalPower = true;
            return;
          }

          if (uiMode == UI_NORMAL) {
            uiMode = UI_SETTINGS;
            menuLevel = LEVEL_MAIN;
          } else if (uiMode == UI_SETTINGS) {
            if (menuLevel == LEVEL_MAIN) {
              if (mainMenuIdx == 8)
                uiMode = UI_NORMAL;        // EXIT
              else if (mainMenuIdx == 4) { // BLE DISPLAY
                uiMode = UI_DISP_MODE;
                subMenuIdx = 0;
                lastMatrixStr = "";
                if (!btInitialized) {
                  SerialBT.begin("BLE_DISPLAY");
                  btInitialized = true;
                }
                lcd.noBacklight();
                lcd.clear();
              } else {
                menuLevel = LEVEL_SUB;
                subMenuIdx = 0;
              }
            } else {
              // Sub Menu Logic
              int maxIdx = 0;
              if (mainMenuIdx == 0)
                maxIdx = 5;
              else if (mainMenuIdx == 1)
                maxIdx = 7;
              else if (mainMenuIdx == 2)
                maxIdx = 7;
              else if (mainMenuIdx == 3)
                maxIdx = 3;
              else if (mainMenuIdx == 5)
                maxIdx = 3;
              else if (mainMenuIdx == 6)
                maxIdx = 2;
              else if (mainMenuIdx == 7)
                maxIdx = 3;

              if (subMenuIdx == maxIdx)
                menuLevel = LEVEL_MAIN; // BACK
              else {
                if (mainMenuIdx == 3 && subMenuIdx == 2) {
                  servoTestMode = !servoTestMode;
                } else if (mainMenuIdx == 6) {
                  if (subMenuIdx == 0) {
                    distCalcRunning = !distCalcRunning;
                  } else if (subMenuIdx == 1) {
                    distCalcTotal = 0.0;
                    distCalcDisplacement = 0.0;
                    distStartPosAcquired = false;
                  }
                } else if (mainMenuIdx == 7) {
                  // Read-only info, click does nothing
                } else {
                  isEditingSetting = !isEditingSetting;
                }
                if (!isEditingSetting) {
                  settingsDirty = true;
                  buildActivePIDList();
                }
              }
            }
          } else if (uiMode == UI_DISP_MODE) {
            if (subMenuIdx == 4)
              uiMode = UI_NORMAL; // BACK
            else
              isEditingSetting = !isEditingSetting;
          }
        }
        buttonPressTime = 0;
      }
    }
  }
  buttonReading = reading;
}

// ======================= DISPLAY RENDERING =======================
static char scrollMsg[128];

void updateMatrix() {
  if (!globalPower || isMatrixAsleep) {
    if (lastMatrixStr != "") {
      matrix.displayClear();
      lastMatrixStr = "";
    }
    return;
  }

  // 1. Independent Intensity Update (Hardware only)
  static int lastActiveIntens = -1;
  int curIntens = (uiMode == UI_DISP_MODE) ? dispBrightness : matrixBrightness;
  if (curIntens != lastActiveIntens) {
    matrix.setIntensity(constrain(curIntens, 0, 15));
    lastActiveIntens = curIntens;
  }

  // 1b. Cricket Push Notification Override (Matrix)
  if (cricketPushActive && (cricketPushTarget == 0 || cricketPushTarget == 1)) {
    static String lastPushMatrix = "";
    if (cricketPushMsg != lastPushMatrix) {
      strcpy(scrollMsg, cricketPushMsg.c_str());
      matrix.displayText(scrollMsg, PA_CENTER, 30, 0, PA_SCROLL_LEFT,
                         PA_SCROLL_LEFT);
      lastPushMatrix = cricketPushMsg;
    }
    return;
  }

  if (uiMode == UI_DISP_MODE) {
    // BLE DISPLAY MODE: Only handle BT and Matrix
    if (SerialBT.available()) {
      String incoming = "";
      incoming.reserve(128);
      unsigned long start = millis();
      while (SerialBT.available() && (millis() - start < 20)) {
        char c = SerialBT.read();
        if (c == '\n' || c == '\r')
          break;
        incoming += c;
      }
      if (incoming.length() > 0) {
        btText = incoming;
        btText.trim();
        lastMatrixStr = "";
        settingsDirty = true;
        lastActivityTime = millis(); // Wake up/Reset sleep timer on new message
      }
    }

    static int lBrt = -1, lSpd = -1, lDir = -1, lScr = -1;
    if (btText != lastMatrixStr || dispBrightness != lBrt ||
        dispSpeed != lSpd || dispDirection != lDir ||
        (int)dispScrollEn != lScr) {
      matrix.setIntensity(constrain(dispBrightness, 0, 15));
      strcpy(scrollMsg, btText.c_str());

      if (dispScrollEn) {
        textEffect_t dir =
            (dispDirection == 0) ? PA_SCROLL_LEFT : PA_SCROLL_RIGHT;
        matrix.displayText(scrollMsg, PA_CENTER, dispSpeed, 100, dir, dir);
      } else {
        matrix.displayClear();
        matrix.displayText(scrollMsg, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
      }

      lastMatrixStr = btText;
      lBrt = dispBrightness;
      lSpd = dispSpeed;
      lDir = dispDirection;
      lScr = (int)dispScrollEn;
      lastActivityTime = millis(); // Keep awake while settings are adjusted
    }
    return;
  }

  if (matrixMode == 1) { // Strobe Logic
    static bool lastStrobe = false;
    static unsigned long strobeTimer = 0;
    if (millis() - strobeTimer > 100) {
      strobeTimer = millis();
      lastStrobe = !lastStrobe;
      if (lastStrobe)
        matrix.print("########");
      else
        matrix.displayClear();
    }
    return;
  }

  if (!matrixPower || matrixTarget == 0) {
    if (lastMatrixStr != "") {
      matrix.displayClear();
      lastMatrixStr = "";
    }
    return;
  }

  String val = getParamStr(matrixTarget);
  if (val != lastMatrixStr) {
    strcpy(scrollMsg, val.c_str());
    if (val.length() > 6 && matrixTarget != 19) {
      matrix.displayText(scrollMsg, PA_CENTER, 30, 0, PA_SCROLL_LEFT,
                         PA_SCROLL_LEFT);
    } else {
      matrix.displayText(scrollMsg, PA_CENTER, 50, 0, PA_PRINT, PA_NO_EFFECT);
    }
    lastMatrixStr = val;
  }
}

void updateOLED() {
  if (!globalPower || isOledAsleep) {
    oled.clearDisplay();
    oled.display();
    return;
  }

  oled.clearDisplay();
  oled.setTextColor(WHITE);

  // Cricket Push Notification Override (OLED)
  if (cricketPushActive && (cricketPushTarget == 0 || cricketPushTarget == 2)) {
    oled.setTextSize(1);
    oled.setCursor(0, 5);
    oled.print("** CRICKET **");
    oled.setTextSize(1);
    oled.setCursor(0, 20);
    // Word-wrap the push message across multiple lines
    String msg = cricketPushMsg;
    int y = 20;
    while (msg.length() > 0 && y < 60) {
      int len = msg.length() > 21 ? 21 : msg.length();
      oled.setCursor(0, y);
      oled.print(msg.substring(0, len));
      msg = msg.substring(len);
      y += 10;
    }
    oled.display();
    return;
  }

  if (uiMode == UI_NORMAL) {
    if (oledParamPower) {
      oled.setTextSize(1);
      oled.setCursor(0, 5);
      oled.print(getParamName(oledTarget));
      String val = getParamStr(oledTarget);
      // Auto-resize long values
      int16_t x1, y1;
      uint16_t w, h;
      oled.setTextSize(3);
      oled.getTextBounds(val, 0, 0, &x1, &y1, &w, &h);
      if (w > 120)
        oled.setTextSize(2); // Drop to size 2 if too long

      oled.setCursor((128 - w) / 2, 30);
      oled.print(val);
    } else {
      oled.setTextSize(1);
      oled.setCursor(0, 0);
      oled.print("-- STATUS --");

      oled.setCursor(0, 10);
      oled.print("NET: ");
      if (wifiConnected) {
        oled.print("Wi-Fi ");
        oled.print(internetConnected ? "(INT)" : "(!INT)");
      } else {
        oled.print("OFF");
      }

      oled.setCursor(0, 20);
      oled.print("GPS: ");
      if (gps.location.isValid()) {
        oled.print("FIX(");
        oled.print(gps.satellites.value());
        oled.print(")");
      } else {
        oled.print("NO FIX");
      }

      oled.setCursor(0, 30);
      oled.print("BLE: ");
      oled.print(obdConnected ? "OK" : "WAIT");

      oled.setCursor(0, 40);
      oled.print("MTX: ");
      oled.print(matrixPower ? getParamName(matrixTarget) : "OFF");

      oled.setCursor(0, 50);
      oled.print("SRV: ");
      oled.print(servoPower ? getParamName(servoTarget) : "OFF");
    }
  } else if (uiMode == UI_SETTINGS) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    if (menuLevel == LEVEL_MAIN) {
      oled.print("- SETTINGS -");
      const char *menuLabels[] = {"OLED",     "LCD",         "MATRIX",
                                  "SERVO",    "BLE DISPLAY", "CRICKET",
                                  "DISTANCE", "WIFI",        "EXIT"};
      int totalItems = 9;
      int visibleItems = 6;
      int startItem = 0;
      if (mainMenuIdx >= visibleItems)
        startItem = mainMenuIdx - visibleItems + 1;
      for (int i = 0; i < visibleItems && (startItem + i) < totalItems; i++) {
        int idx = startItem + i;
        oled.setCursor(5, 15 + i * 8);
        if (idx == mainMenuIdx)
          oled.print("> ");
        else
          oled.print("  ");
        oled.print(menuLabels[idx]);
      }
    } else {
      oled.print("- ");
      const char *subTitles[] = {"OLED", "LCD",     "MATRIX",   "SERVO",
                                 "",     "CRICKET", "DISTANCE", "WIFI"};
      if (mainMenuIdx <= 7)
        oled.print(subTitles[mainMenuIdx]);
      oled.print(" -");
      if (isEditingSetting)
        oled.print(" *");

      int start = (subMenuIdx > 4) ? subMenuIdx - 4 : 0;
      for (int i = 0; i < 5; i++) {
        int item = start + i;
        oled.setCursor(0, 15 + i * 10);
        if (item == subMenuIdx)
          oled.print(isEditingSetting ? "* " : "-> ");
        else
          oled.print("   ");

        if (mainMenuIdx == 0) { // OLED
          switch (item) {
          case 0:
            oled.print("parameter: ");
            oled.print(getParamName(oledTarget));
            break;
          case 1:
            oled.print("Value: ");
            oled.print(oledParamPower ? "Parameter" : "STATUS");
            break;
          case 2:
            oled.print("Sleep: ");
            oled.print(oledSleepEn ? "ON" : "OFF");
            break;
          case 3:
            oled.print("Timeout M: ");
            oled.print(oledSleepTime / 60);
            break;
          case 4:
            oled.print("Timeout S: ");
            oled.print(oledSleepTime % 60);
            break;
          case 5:
            oled.print("BACK");
            break;
          default:
            i = 6;
            break;
          }
        } else if (mainMenuIdx == 1) { // LCD
          switch (item) {
          case 0:
            oled.print("Mode: ");
            oled.print(lcdMode == 0 ? "Parameters" : "TITLE");
            break;
          case 1:
            oled.print("Line1: ");
            oled.print(getParamName(lcdRow1Target));
            break;
          case 2:
            oled.print("Line2: ");
            oled.print(getParamName(lcdRow2Target));
            break;
          case 3:
            oled.print("Power: ");
            oled.print(lcdPower ? "ON" : "OFF");
            break;
          case 4:
            oled.print("Sleep: ");
            oled.print(lcdSleepEn ? "ON" : "OFF");
            break;
          case 5:
            oled.print("Timeout M: ");
            oled.print(lcdSleepTime / 60);
            break;
          case 6:
            oled.print("Timeout S: ");
            oled.print(lcdSleepTime % 60);
            break;
          case 7:
            oled.print("BACK");
            break;
          default:
            i = 8;
            break;
          }
        } else if (mainMenuIdx == 2) { // MATRIX
          switch (item) {
          case 0:
            oled.print("parameter: ");
            oled.print(getParamName(matrixTarget));
            break;
          case 1:
            oled.print("Power: ");
            oled.print(matrixPower ? "ON" : "OFF");
            break;
          case 2:
            oled.print("Mode: ");
            oled.print(matrixMode == 0 ? "NORM" : "STRB");
            break;
          case 3:
            oled.print("Brightness: ");
            oled.print(matrixBrightness);
            break;
          case 4:
            oled.print("Sleep: ");
            oled.print(matrixSleepEn ? "ON" : "OFF");
            break;
          case 5:
            oled.print("Timeout M: ");
            oled.print(matrixSleepTime / 60);
            break;
          case 6:
            oled.print("Timeout S: ");
            oled.print(matrixSleepTime % 60);
            break;
          case 7:
            oled.print("BACK");
            break;
          default:
            i = 8;
            break;
          }
        } else if (mainMenuIdx == 3) { // SERVO
          switch (item) {
          case 0:
            oled.print("parameter: ");
            oled.print(getParamName(servoTarget));
            break;
          case 1:
            oled.print("Power: ");
            oled.print(servoPower ? "ON" : "OFF");
            break;
          case 2:
            oled.print("link to knob: ");
            oled.print(servoTestMode ? "ON" : "OFF");
            break;
          case 3:
            oled.print("BACK");
            break;
          default:
            i = 5;
            break;
          }
        } else if (mainMenuIdx == 5) { // CRICKET
          const char *pushNames[] = {"ALL", "MATRIX", "OLED", "LCD", "NONE"};
          switch (item) {
          case 0:
            oled.print("Team1: ");
            oled.print(popularTeams[cricketTeam1Idx]);
            break;
          case 1:
            oled.print("Team2: ");
            oled.print(popularTeams[cricketTeam2Idx]);
            break;
          case 2:
            oled.print("Push: ");
            oled.print(pushNames[cricketPushTarget]);
            break;
          case 3:
            oled.print("BACK");
            break;
          default:
            i = 5;
            break;
          }
        } else if (mainMenuIdx == 6) { // DISTANCE
          switch (item) {
          case 0:
            oled.print(distCalcRunning ? "[STOP]" : "[START]");
            break;
          case 1:
            oled.print("[RESET]");
            break;
          case 2:
            oled.print("BACK");
            break;
          default:
            i = 5;
            break;
          }
          // Show live stats below menu
          if (i == 0) {
            oled.setCursor(70, 15);
            char dbuf[16];
            sprintf(dbuf, "%.2fkm", distCalcTotal / 1000.0);
            oled.print(dbuf);
          }
        } else if (mainMenuIdx == 7) { // WIFI
          switch (item) {
          case 0:
            oled.print("WiFi: ");
            oled.print(wifiConnected ? "OK" : "...");
            break;
          case 1:
            oled.print("SSID: ");
            oled.print(wifiSSID.substring(0, 10));
            break;
          case 2:
            oled.print("IP: ");
            oled.print(localIPStr);
            break;
          case 3:
            oled.print("BACK");
            break;
          default:
            i = 5;
            break;
          }
        }
      }
    }
  } else if (uiMode == UI_DISP_MODE) {
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("-- BLE DISPLAY --");
    oled.setCursor(0, 11);
    oled.print("BT: ");
    oled.print(btInitialized ? "ON" : "OFF");
    int start = (subMenuIdx > 3) ? subMenuIdx - 3 : 0;
    for (int i = 0; i < 4; i++) {
      int item = start + i;
      oled.setCursor(0, 22 + i * 10);
      if (item == subMenuIdx)
        oled.print(isEditingSetting ? "* " : "-> ");
      else
        oled.print("   ");
      switch (item) {
      case 0:
        oled.print("Brighness: ");
        oled.print(dispBrightness);
        break;
      case 1:
        oled.print("Speed: ");
        oled.print(dispSpeed);
        break;
      case 2:
        oled.print("Direction: ");
        oled.print(dispDirection == 0 ? "Left" : "Right");
        break;
      case 3:
        oled.print("Scroll: ");
        oled.print(dispScrollEn ? "ON" : "OFF");
        break;
      case 4:
        oled.print("BACK");
        break;
      }
    }
  }
  oled.display();
}

void updateLCD() {
  if (!globalPower || !lcdPower || isLcdAsleep || uiMode == UI_DISP_MODE) {
    lcd.noBacklight();
    lcd.clear();
    return;
  }
  lcd.backlight();

  // Cricket Push Notification Override (LCD)
  if (cricketPushActive && (cricketPushTarget == 0 || cricketPushTarget == 3)) {
    lcd.setCursor(0, 0);
    lcd.print("** CRICKET **   ");
    lcd.setCursor(0, 1);

    String msg = cricketPushMsg;
    if (msg.length() > 16) {
      int idx = (millis() / 300) % (msg.length() + 3);
      String pad = msg + "   " + msg + "                ";
      lcd.print(pad.substring(idx, idx + 16));
    } else {
      lcd.print((msg + "                ").substring(0, 16));
    }
    return;
  }

  if (lcdMode == 1) { // Matrix Title mode
    lcd.setCursor(0, 0);
    String title = getParamName(matrixTarget) + "          ";
    lcd.print(title.substring(0, 16));
    lcd.setCursor(0, 1);
    lcd.print("                ");
  } else {
    lcd.setCursor(0, 0);
    String l1 = (lcdRow1Target >= 27) ? getParamStr(lcdRow1Target)
                                      : getParamName(lcdRow1Target) + ":" +
                                            getParamStr(lcdRow1Target);
    lcd.print((l1 + "                ").substring(0, 16));

    lcd.setCursor(0, 1);
    String l2 = (lcdRow2Target >= 27) ? getParamStr(lcdRow2Target)
                                      : getParamName(lcdRow2Target) + ":" +
                                            getParamStr(lcdRow2Target);
    lcd.print((l2 + "                ").substring(0, 16));
  }
}

void updateServo() {
  if (!globalPower || !servoPower || servoTarget == 0) {
    if (myServo.attached())
      myServo.detach();
    return;
  }

  if (!myServo.attached()) {
    myServo.attach(SERVO_PIN, 500, 2400);
  }

  int angle = 0;
  if (servoTestMode) {
    int potVal = analogRead(POT_PIN);
    // Direct reverse follow: Pot high (4095) -> Servo Low (0), Pot low (0) ->
    // Servo High (180)
    angle = map(potVal, 0, 4095, 0, 180);
  } else {
    switch (servoTarget) {
    case 1:
      angle = map(engineRPM, 0, 6000, 5, 180);
      break;
    case 2:
      angle = map(vehicleSpeed, 0, 200, 0, 180);
      break;
    case 3:
      angle = map(coolantTemp, -40, 150, 0, 180);
      break;
    case 4:
      angle = map(intakeTemp, -40, 150, 0, 180);
      break;
    case 5:
      angle = map(ambientTemp, -40, 150, 0, 180);
      break;
    case 6:
      angle = map(absLoad, 0, 100, 0, 180);
      break;
    case 7:
      angle = map(pedalPos, 0, 100, 0, 180);
      break;
    case 8:
      angle = map(controlVoltage * 10, 0, 150, 0, 180);
      break;
    case 9:
      angle = map(timeSinceStart, 0, 3600, 0, 180);
      break;
    case 16:
      angle = map((int)gps.speed.kmph(), 0, 60, 0, 180);
      break;
    }
  }
  angle = constrain(angle, 0, 180);
  myServo.write(180 - angle); // Reversed travel direction
}

// ======================= BOOT & SETUP =======================
void bootSequence() {
  // Removed servo rotation sweep during startup
  delay(500);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("INITIALIZING...");

  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(0, 20);
  oled.print("INITIALIZING");
  oled.display();

  matrix.displayClear();
}

void sendOBDCommandBlocking(String cmd) {
  rawIdx = 0;
  rawResponse[0] = '\0';
  responseReady = false;
  ioChar->writeValue(cmd + "\r", false);
  unsigned long start = millis();
  while (!responseReady && millis() - start < 1500) {
    delay(5);
  }
}

TaskHandle_t networkTaskHandle;

void networkTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());

  unsigned long lastCricketFetch = 0;

  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnected) {
        wifiConnected = true;
        localIPStr = WiFi.localIP().toString();
        netFetchPending = true;
      }
    } else {
      wifiConnected = false;
      localIPStr = "Disconnected";
      WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    if (netFetchPending && wifiConnected) {
      double fetchLat = gps.location.isValid() ? gps.location.lat() : 12.9716;
      double fetchLon = gps.location.isValid() ? gps.location.lng() : 77.5946;

      {
        HTTPClient http;
        WiFiClientSecure client;
        client.setInsecure();

        String nominatimUrl = "https://nominatim.openstreetmap.org/reverse?lat=" +
                              String(fetchLat, 6) +
                              "&lon=" + String(fetchLon, 6) + "&format=json";
        if (http.begin(client, nominatimUrl)) {
          http.setUserAgent("OBDMeterBox/1.0 (contact@example.com)");
          int httpCode = http.GET();
          if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();

            String postcode = findNestedJsonValue(payload, "address", "postcode");
            if (postcode == "")
              postcode = findJsonValue(payload, "postcode");
            if (postcode != "")
              postcodeStr = postcode;

            String road = findNestedJsonValue(payload, "address", "road");
            String suburb = findNestedJsonValue(payload, "address", "suburb");
            String city = findNestedJsonValue(payload, "address", "city");
            if (city == "")
              city = findNestedJsonValue(payload, "address", "town");
            if (city == "")
              city = findNestedJsonValue(payload, "address", "village");

            String constructedAddress = "";
            if (road != "")
              constructedAddress += road;
            if (suburb != "") {
              if (constructedAddress != "")
                constructedAddress += ", ";
              constructedAddress += suburb;
            }
            if (city != "") {
              if (constructedAddress != "")
                constructedAddress += ", ";
              constructedAddress += city;
            }

            if (constructedAddress != "") {
              addressStr = constructedAddress;
            } else {
              String dispName = findJsonValue(payload, "display_name");
              if (dispName != "") {
                addressStr = dispName;
              }
            }
          }
          http.end();
        }
      }

      {
        HTTPClient http;
        WiFiClientSecure client;
        client.setInsecure();

        String aqicnUrl =
            "https://api.waqi.info/feed/geo:" + String(fetchLat, 6) + ";" +
            String(fetchLon, 6) +
            "/?token=00847560f5cfaca677fa696c7c39f009d6915076";
        if (http.begin(client, aqicnUrl)) {
          int httpCode = http.GET();
          if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();

            String aqiStr = findNestedJsonValue(payload, "data", "aqi");
            if (aqiStr != "")
              aqiValue = aqiStr.toInt();

            String pm25Str = findNestedJsonValue(payload, "pm25", "v");
            if (pm25Str != "")
              pm25Value = pm25Str.toFloat();

            String pm10Str = findNestedJsonValue(payload, "pm10", "v");
            if (pm10Str != "")
              pm10Value = pm10Str.toFloat();

            String coStr = findNestedJsonValue(payload, "co", "v");
            if (coStr != "")
              coValue = coStr.toFloat();

            String no2Str = findNestedJsonValue(payload, "no2", "v");
            if (no2Str != "")
              no2Value = no2Str.toFloat();

            String o3Str = findNestedJsonValue(payload, "o3", "v");
            if (o3Str != "")
              o3Value = o3Str.toFloat();
          }
          http.end();
        }
      }

      netFetchPending = false;
      lastNetRequestTime = millis();
    }

    if (wifiConnected &&
        (millis() - lastCricketFetch > 60000 || lastCricketFetch == 0)) {
      HTTPClient http;
      WiFiClientSecure client;
      client.setInsecure();

      String cricUrl =
          "https://api.cricapi.com/v1/"
          "currentMatches?apikey=bb86bb57-c127-40d2-9991-30af34871b35&offset=0";
      if (http.begin(client, cricUrl)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
          internetConnected = true;
          String payload = http.getString();
          parseCricketScore(payload);
        } else {
          internetConnected = false;
        }
        http.end();
      } else {
        internetConnected = false;
      }
      lastCricketFetch = millis();
    }

    if (millis() - lastNetRequestTime > 600000) {
      netFetchPending = true;
    }

    checkLocationChanges();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

TaskHandle_t obdTaskHandle;

void obdTask(void *pvParameters) {
  for (;;) {
    if (!obdConnected) {
      BLEDevice::init("");
      bleClient = BLEDevice::createClient();
      while (!bleClient->connect(obdAddr)) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
      }

      BLERemoteService *s = bleClient->getService(serviceUUID);
      if (s) {
        ioChar = s->getCharacteristic(ioUUID);
        if (ioChar) {
          ioChar->registerForNotify(notifyCallback);
        }
      }

      sendOBDCommandBlocking("ATZ");
      vTaskDelay(1500 / portTICK_PERIOD_MS);
      sendOBDCommandBlocking("ATE0");
      sendOBDCommandBlocking("ATH0");
      sendOBDCommandBlocking("ATL0");
      sendOBDCommandBlocking("ATS0");
      sendOBDCommandBlocking("ATAT1");
      sendOBDCommandBlocking("ATST19");
      sendOBDCommandBlocking("ATSP0");

      bool ecuOn = false;
      while (!ecuOn) {
        sendOBDCommandBlocking("010C");
        String resp = String(rawResponse);
        if (resp.indexOf("410C") != -1) {
          ecuOn = true;
        } else {
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
      }
      obdConnected = true;
      buildActivePIDList();
    } else {
      processAsyncOBD();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_GND_PIN, OUTPUT);
  digitalWrite(BUTTON_GND_PIN, LOW);
  pinMode(BUTTON_READ_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(200000); // 200kHz is safer for multiple displays

  lcd.init();
  lcd.backlight();

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Failed");
  }

  matrix.begin();

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);

  loadSettings(); // Load saved settings FIRST before boot display
  lastActivityTime = millis();
  lastRawPot = analogRead(POT_PIN);

  bootSequence();
  xTaskCreatePinnedToCore(obdTask, "OBDTask", 8192, NULL, 1, &obdTaskHandle, 1);
  xTaskCreatePinnedToCore(networkTask, "NetTask", 16384, NULL, 1,
                          &networkTaskHandle, 0);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
}

// ======================= MAIN LOOP =======================
unsigned long lastDisplayUpdate = 0;

void loop() {
  // 1. Process Matrix Animation at maximum frequency (Priority)
  if (matrix.displayAnimate()) {
    if (matrixTarget == 27 || matrixTarget == 28 ||
        (uiMode == UI_DISP_MODE && dispScrollEn)) {
      matrix.displayReset();
    }
  }

  // 2. Background parsing only when NOT in BLE DISPLAY to save CPU
  if (uiMode != UI_DISP_MODE) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
  }

  // 2b. Distance Calculator update (every 1s)
  static unsigned long lastDistUpdate = 0;
  if (millis() - lastDistUpdate > 1000) {
    lastDistUpdate = millis();
    updateDistanceCalculator();
  }

  // 2c. Push notification expiry
  if (cricketPushActive && millis() - cricketPushStartTime > 5000) {
    cricketPushActive = false;
    lastMatrixStr = "";
  }

  // 3. User interaction
  checkInput();

  // 4. Low-frequency rendering loop
  if (millis() - lastDisplayUpdate > 100) {
    lastDisplayUpdate = millis();

    updateMatrix(); // updateMatrix handles its own exit if in DISP_MODE

    // Stop all secondary displays in BLE DISPLAY mode for pure performance
    if (uiMode != UI_DISP_MODE) {
      updateOLED();
      updateLCD();
      updateServo();
    } else {
      // Keep OLED active only if we are currently looking at the settings menu
      updateOLED();
      // LCD and Servo explicitly skipped
    }

    // Periodic save to flash
    if (settingsDirty && millis() - lastSaveTime > 2000) {
      saveSettings();
      settingsDirty = false;
      lastSaveTime = millis();
    }
  }
}
