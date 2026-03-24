#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ctype.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// OLED display size for SSD1306 module
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Create display object using I2C (Wire)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MPU6050 object (default I2C address 0x68)
Adafruit_MPU6050 mpu;
bool mpuReady = false;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float filteredX = 0, filteredY = 0;
float lagFactor = 0.08f; // Lower = more lag
float faceOffsetX = 0, faceOffsetY = 0;
float faceVelX = 0, faceVelY = 0;
float gyroPush = 1.9f;    // how strongly rotation pushes the face
float springBack = 0.08f; // pull back toward center
float damping = 0.84f;    // motion decay per frame
bool easeBackToCenter = true; // Set to false to disable spring back
bool rotateWithMotion = true; // Set to false to disable face rotation with motion
bool easeRotateBackToCenter = true; // Set to false to disable angular spring back
float faceAngle = 0.0f;       // face rotation angle in radians
float faceAngVel = 0.0f;      // angular velocity in radians/frame
float gyroRotatePush = 0.008f; // how strongly gyro Z pushes face rotation
float angleSpringBack = 0.06f;
float angleDamping = 0.86f;
float maxFaceAngle = 0.50f;   // ~28.6 degrees
unsigned long lastGyroPrintMs = 0;

WebServer server(80);
String incomingText = "";

const char *apSsid = "RobotFace";
const char *apPassword = "robotface";

const char htmlPage[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Face</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 24px; }
    input, button { font-size: 18px; padding: 10px; }
    input { width: 100%; box-sizing: border-box; margin-bottom: 12px; }
  </style>
</head>
<body>
  <h2>Send text to robot</h2>
  <form action="/send" method="get">
    <input name="text" maxlength="120" placeholder="Type text here" required>
    <button type="submit">Send</button>
  </form>
</body>
</html>
)HTML";

const int buzzerPin = 27;
const int buzzerChannel = 0;
const int buzzerResolution = 8;
const int charBeepFrequencyHz = 440;
const unsigned long charBeepOnMs = 50;
const unsigned long charBeepGapMs = 35;
const unsigned long spacePauseMs = 150;
const int maxSpeechEvents = 400;

char speechEventQueue[maxSpeechEvents]; // 'B' = beep, 'S' = silent pause
int speechEventCount = 0;
bool charBeepToneOn = false;
bool charBeepInGap = false;
char activeSpeechEvent = '\0';
unsigned long charBeepPhaseStartMs = 0;

bool enqueueSpeechEvent(char eventType) {
  if (speechEventCount >= maxSpeechEvents) {
    return false;
  }
  speechEventQueue[speechEventCount++] = eventType;
  return true;
}

void popSpeechEvent() {
  if (speechEventCount <= 0) {
    return;
  }
  for (int i = 1; i < speechEventCount; i++) {
    speechEventQueue[i - 1] = speechEventQueue[i];
  }
  speechEventCount--;
}

void playStartupBeep() {
  ledcWriteTone(buzzerChannel, 1800);
  delay(80);
  ledcWriteTone(buzzerChannel, 0);
}

float getCharacterFrequency(char c) {
  float baseA = 440.0f;
  float semitone = pow(2.0f, 1.0f / 12.0f);
  
  if (c >= 'a' && c <= 'z') {
    int noteOffset = c - 'a';
    return baseA * pow(semitone, (float)noteOffset);
  } else if (c >= 'A' && c <= 'Z') {
    int noteOffset = c - 'A';
    return baseA * 2.0f * pow(semitone, (float)noteOffset);
  } else if (c >= '0' && c <= '9') {
    int digit = c - '0';
    int noteOffset = (digit == 0) ? 9 : (digit - 1);
    return (baseA * 0.5f) * pow(semitone, (float)noteOffset);
  }
  
  return 0.0f;
}

boolean isCharacterBeepable(char c) {
  return isalnum((unsigned char)c);
}

boolean isCharacterSpace(char c) {
  return c == ' ';
}

boolean isPlayingNotes() {
  return charBeepToneOn || speechEventCount > 0;
}

void serviceCharBeeps() {
  if (activeSpeechEvent == '\0' && speechEventCount <= 0) {
    charBeepInGap = false;
    return;
  }

  unsigned long now = millis();

  if (activeSpeechEvent == '\0' && speechEventCount > 0) {
    activeSpeechEvent = speechEventQueue[0];

    if (isCharacterBeepable(activeSpeechEvent)) {
      float freq = getCharacterFrequency(activeSpeechEvent);
      ledcWriteTone(buzzerChannel, (uint32_t)freq);
      charBeepToneOn = true;
      charBeepInGap = false;
      charBeepPhaseStartMs = now;
      return;
    }

    if (isCharacterSpace(activeSpeechEvent)) {
      charBeepToneOn = false;
      charBeepInGap = false;
      charBeepPhaseStartMs = now;
      return;
    }

    popSpeechEvent();
    activeSpeechEvent = '\0';
    return;
  }

  if (isCharacterSpace(activeSpeechEvent)) {
    if (now - charBeepPhaseStartMs >= spacePauseMs) {
      popSpeechEvent();
      activeSpeechEvent = '\0';
    }
    return;
  }

  if (isCharacterBeepable(activeSpeechEvent) && charBeepToneOn && now - charBeepPhaseStartMs >= charBeepOnMs) {
    ledcWriteTone(buzzerChannel, 0);
    charBeepToneOn = false;
    charBeepInGap = true;
    charBeepPhaseStartMs = now;
    return;
  }

  if (isCharacterBeepable(activeSpeechEvent) && charBeepInGap && now - charBeepPhaseStartMs >= charBeepGapMs) {
    charBeepInGap = false;
    popSpeechEvent();
    activeSpeechEvent = '\0';
  }
}

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleSend() {
  if (!server.hasArg("text")) {
    server.send(400, "text/plain", "Missing 'text' parameter.");
    return;
  }

  incomingText = server.arg("text");

  if (incomingText.length() > 120) {
    incomingText = incomingText.substring(0, 120);
  }

  Serial.print("Received text: ");
  Serial.println(incomingText);

  for (size_t i = 0; i < incomingText.length(); i++) {
    char c = incomingText[i];
    if (isCharacterBeepable(c) || isCharacterSpace(c)) {
      enqueueSpeechEvent(c);
    }
  }

  String response = "Saved: " + incomingText + "\\nOpen / to send another.";
  server.send(200, "text/plain", response);
}

void setupPhoneTextServer() {
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(apSsid, apPassword);

  if (!apStarted) {
    Serial.println("Failed to start AP");
    return;
  }

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP ready. SSID: ");
  Serial.println(apSsid);
  Serial.print("Password: ");
  Serial.println(apPassword);
  Serial.print("Open on phone: http://");
  Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/send", handleSend);
  server.begin();
  Serial.println("Web text server started.");
}

void scanI2CBus(TwoWire &bus, const char *label) {
  Serial.print("I2C scan on ");
  Serial.print(label);
  Serial.println("...");
  int foundCount = 0;
  for (uint8_t address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    uint8_t error = bus.endTransmission();
    if (error == 0) {
      Serial.print("  device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      foundCount++;
    }
  }
  if (foundCount == 0) Serial.println("  No devices found.");
}

// Simple 2D point type for mouth vertices and Bézier control points
struct Point { float x, y; };

// 8 anchor points for mouth silhouette plus 16 cubic control points (2 per segment)
Point anchors[8];
Point controls[16];

// Eye anchor points and control points (left and right eyes)
Point eyeAnchorsL[8];
Point eyeControlsL[16];
Point eyeAnchorsR[8];
Point eyeControlsR[16];

// Face center position in screen space (move this to move the entire face)
float faceCx = 64;
float faceCy = 40;

// Relative offsets of the mouth and eyes from the face center (in local coordinates)
// Used for both positioning and future rotation calculations
float mouthOffsetX = 0;    // mouth directly below face center
float mouthOffsetY = 4;

float eyeLeftOffsetX = -44;
float eyeLeftOffsetY = -8;

float eyeRightOffsetX = 44;
float eyeRightOffsetY = -8;

// Derived screen-space positions (updated in updateMouth/Eyes)
float mouthCx = faceCx + mouthOffsetX;
float mouthCy = faceCy + mouthOffsetY;

float eyeLcx = faceCx + eyeLeftOffsetX;
float eyeLcy = faceCy + eyeLeftOffsetY;

float eyeRcx = faceCx + eyeRightOffsetX;
float eyeRcy = faceCy + eyeRightOffsetY;

// Shape keys (preset mouth vertex layout). We use 8 points for an octagon-style mouth.
// Defined in LOCAL coordinates relative to mouth center (mouthCx, mouthCy)
Point shapeA[8] = {
  {-9, 0}, {-7, -2}, {-3, -4}, {3, -4}, {7, -2}, {9, 0}, {7, 2}, {-7, 2}
};

Point shapeB[8] = {
  {-9, 0}, {-7, -4}, {-4, -8}, {4, -8}, {7, -4}, {9, 0}, {7, 4}, {-7, 4}
};

Point shapeC[8] = {
  {-9, 2}, {-7, 4}, {-3, 6}, {3, 6}, {7, 4}, {9, 2}, {7, 0}, {-7, 0}
};

Point smile[8] = {
  {15, -12}, {10, -12}, {0, -12}, {-10, -12}, {-15, -12}, {-10.6f, -1.4f}, {0, 3}, {10.6f, -1.4f}
};

Point frown[8] = {
  {15, -12}, {10, -12}, {0, -12}, {-10, -12}, {-15, -12}, {-10.6f, -22.6f}, {0, -27}, {10.6f, -22.6f}
};

Point closed[8] = {
  {15, -5}, {10, -5}, {0, -5}, {-10, -5}, {-15, -5}, {-10.6f, -5}, {0, -5}, {10.6f, -5}
};

Point neutral[8] = {
  {15, -9}, {10, -10}, {0, -11}, {-10, -10}, {-15, -9}, {-10.6f, -4}, {0, -3}, {10.6f, -4}
};

// Eye shape presets (relative to eye center, 8 points per eye shape)
Point eyeOpen[8] = {
  {-8, 0}, {-6, -6}, {0, -8}, {6, -6}, {8, 0}, {6, 6}, {0, 8}, {-6, 6}
};

Point eyeClosed[8] = {
  {-8, -1}, {-6, -1}, {0, -1}, {6, -1}, {8, -1}, {6, 1}, {0, 1}, {-6, 1}
};

Point eyeAngry[8] = {
  {-8, 1}, {-6, -4}, {0, -7}, {6, -4}, {8, 1}, {6, 5}, {0, 6}, {-6, 5}
};

Point eyeSad[8] = {
  {-8, -2}, {-6, 2}, {0, 5}, {6, 2}, {8, -2}, {6, -6}, {0, -7}, {-6, -6}
};

// Animation phase (incremented each loop)
float tAnim = 0;

// Random mouth animation state
float mouthPhase = 0.0f;           // Current blend phase (0 to 1)
float mouthTargetPhase = 0.5f;     // Target blend phase
float mouthTransitionTime = 0.0f;  // Time within current transition
float mouthTransitionDuration = 1.5f;  // Duration of transition in seconds

// Linear interpolation helper for two points
Point lerp(Point a, Point b, float t) {
  return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
}

// Rotate a local point (around origin) by angle in radians
Point rotatePoint(Point p, float angle) {
  float c = cos(angle);
  float s = sin(angle);
  return {
    p.x * c - p.y * s,
    p.x * s + p.y * c
  };
}

// Blend between two shape arrays (8 points each) by factor x (0 to 1)
// Writes result to destAnchors. Transitions from shapeA (x=0) to shapeB (x=1)
void blendShapesInto(Point shapeA[8], Point shapeB[8], Point destAnchors[8], float x) {
  for (int i = 0; i < 8; i++) {
    destAnchors[i] = lerp(shapeA[i], shapeB[i], x);
  }
}


// Cubic Bézier at parameter t in [0,1], with end points p0/p1 and handles c0/c1
Point cubicBezier(Point p0, Point c0, Point c1, Point p1, float t) {
  Point a = lerp(p0, c0, t); // first linear blend
  Point b = lerp(c0, c1, t);
  Point c = lerp(c1, p1, t);

  Point d = lerp(a, b, t); // second-level blend
  Point e = lerp(b, c, t);

  return lerp(d, e, t); // final point on curve
}

// Update mouth anchor points and control points for smooth curve
void updateMouth() {
  // Recalculate mouth position from face center (with face rotation)
  Point mouthOffsetRot = rotatePoint({mouthOffsetX, mouthOffsetY}, faceAngle);
  mouthCx = faceCx + mouthOffsetRot.x;
  mouthCy = faceCy + mouthOffsetRot.y;

//   // Morph from shapeA -> shapeB -> shapeC by time. This is your keyframe blending.
//   float phase = (sin(tAnim * 0.7f) + 1.0f) / 2.0f; // [0,1]
  
//   if (phase <= 0.5f) {
//     // First half: blend from shapeA to shapeB
//     float x = phase * 2.0f; // remaps [0, 0.5] to [0, 1]
//     blendShapesInto(smile, closed, anchors, x);
//   } else {
//     // Second half: blend from shapeB to shapeC
//     float x = (phase - 0.5f) * 2.0f; // remaps [0.5, 1] to [0, 1]
//     blendShapesInto(closed, smile, anchors, x);
//   }

  float phase;
  if (isPlayingNotes()) {
    phase = (sin(tAnim * 20.0f) + 1.0f) / 2.0f;  // animate when playing notes
  } else {
    phase = 0.05f;  // blend almost completely closed when silent
  }
  
  // Temporary array for blended mouth shape (in local coordinates)
  Point blended[8];
  blendShapesInto(closed, neutral, blended, phase);
  
  // Offset blend result to screen position and store in anchors
  for (int i = 0; i < 8; i++) {
    Point p = rotatePoint(blended[i], faceAngle);
    anchors[i].x = p.x + mouthCx;
    anchors[i].y = p.y + mouthCy;
  }

  // Convert anchors into control points for smooth cubic segments.
  // Uses a Catmull-Rom-like handle scheme: the tangent at each anchor
  // points toward the average of neighbors.
  for (int i = 0; i < 8; i++) {
    Point prev = anchors[(i + 7) % 8];
    Point cur = anchors[i];
    Point next = anchors[(i + 1) % 8];
    float alpha = 0.35f;

    Point cp = {
      cur.x + (next.x - prev.x) * alpha,
      cur.y + (next.y - prev.y) * alpha
    };

    // symmetric in-out handles at each anchor for C2-ish continuity
    controls[i * 2 + 0] = cp;
    controls[i * 2 + 1] = cp;
  }
}

// Update eye anchor points and control points for smooth curve
// Applies blending between two eye shapes and generates control points
void updateEyes() {
  // Recalculate eye positions from face center
  Point eyeLOffsetRot = rotatePoint({eyeLeftOffsetX, eyeLeftOffsetY}, faceAngle);
  Point eyeROffsetRot = rotatePoint({eyeRightOffsetX, eyeRightOffsetY}, faceAngle);
  eyeLcx = faceCx + eyeLOffsetRot.x;
  eyeLcy = faceCy + eyeLOffsetRot.y;
  eyeRcx = faceCx + eyeROffsetRot.x;
  eyeRcy = faceCy + eyeROffsetRot.y;

  // Blink cycle: 3 seconds total
  // Eyes stay open for 2.7 seconds, then blink (close/open) for 0.3 seconds
  float blinkCyclePeriod = 3.0f;
  float blinkFraction = fmod(tAnim, blinkCyclePeriod) / blinkCyclePeriod;  // 0 to 1
  
  float phase;
  if (blinkFraction < 0.9f) {
    // Eyes open for 90% of cycle (2.7 seconds)
    phase = 0.0f;
  } else {
    // Eyes blink for 10% of cycle (0.3 seconds)
    float blinkTime = (blinkFraction - 0.9f) / 0.1f;  // 0 to 1 during blink
    phase = sin(blinkTime * M_PI) * 0.5f + 0.5f;  // smooth sine wave for blink motion
  }
  
  // Temporary arrays for blended shapes (in local coordinates)
  Point blendedL[8], blendedR[8];
  
  // Blend left and right eye shapes
  blendShapesInto(eyeOpen, eyeClosed, blendedL, phase);
  blendShapesInto(eyeOpen, eyeClosed, blendedR, phase);
  
  // Offset blend results to screen positions and store in anchor arrays
  for (int i = 0; i < 8; i++) {
    Point leftP = rotatePoint(blendedL[i], faceAngle);
    eyeAnchorsL[i].x = leftP.x + eyeLcx;
    eyeAnchorsL[i].y = leftP.y + eyeLcy;
    
    Point rightP = rotatePoint(blendedR[i], faceAngle);
    eyeAnchorsR[i].x = rightP.x + eyeRcx;
    eyeAnchorsR[i].y = rightP.y + eyeRcy;
  }

  // Generate control points for left eye
  for (int i = 0; i < 8; i++) {
    Point prev = eyeAnchorsL[(i + 7) % 8];
    Point cur = eyeAnchorsL[i];
    Point next = eyeAnchorsL[(i + 1) % 8];
    float alpha = 0.35f;

    Point cp = {
      cur.x + (next.x - prev.x) * alpha,
      cur.y + (next.y - prev.y) * alpha
    };

    eyeControlsL[i * 2 + 0] = cp;
    eyeControlsL[i * 2 + 1] = cp;
  }

  // Generate control points for right eye
  for (int i = 0; i < 8; i++) {
    Point prev = eyeAnchorsR[(i + 7) % 8];
    Point cur = eyeAnchorsR[i];
    Point next = eyeAnchorsR[(i + 1) % 8];
    float alpha = 0.35f;

    Point cp = {
      cur.x + (next.x - prev.x) * alpha,
      cur.y + (next.y - prev.y) * alpha
    };

    eyeControlsR[i * 2 + 0] = cp;
    eyeControlsR[i * 2 + 1] = cp;
  }
}

// Render an eye shape as an 8-segment connected cubic Bézier path
void drawEyeBezier(Point anchors[8], Point controls[16]) {
  const int samples = 10;

  for (int i = 0; i < 8; i++) {
    Point p0 = anchors[i];
    Point p1 = anchors[(i + 1) % 8];
    Point c0 = controls[i * 2 + 1];
    Point c1 = controls[((i + 1) % 8) * 2 + 0];

    Point prev = p0;

    for (int j = 1; j <= samples; j++) {
      float t = j / float(samples);
      Point p = cubicBezier(p0, c0, c1, p1, t);
      display.drawLine((int)round(prev.x), (int)round(prev.y),
                       (int)round(p.x), (int)round(p.y), WHITE);
      prev = p;
    }
  }
}

// Fill an eye shape using scanline fill
void fillEyePoly(Point anchors[8]) {
  int ymin = SCREEN_HEIGHT, ymax = 0;
  for (int i = 0; i < 8; i++) {
    int y = (int)round(anchors[i].y);
    ymin = min(ymin, y);
    ymax = max(ymax, y);
  }

  for (int y = max(0, ymin); y <= min(SCREEN_HEIGHT - 1, ymax); y++) {
    int intersections[16];
    int ni = 0;

    for (int i = 0; i < 8; i++) {
      Point a = anchors[i];
      Point b = anchors[(i + 1) % 8];
      if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
        float t = (y - a.y) / (b.y - a.y);
        float x = a.x + (b.x - a.x) * t;
        intersections[ni++] = (int)round(x);
      }
    }

    if (ni < 2) continue;
    for (int i = 0; i < ni-1; i++) {
      for (int j = i + 1; j < ni; j++) {
        if (intersections[j] < intersections[i]) {
          int tmp = intersections[i];
          intersections[i] = intersections[j];
          intersections[j] = tmp;
        }
      }
    }

    for (int i = 0; i + 1 < ni; i += 2) {
      int x0 = max(0, min(SCREEN_WIDTH - 1, intersections[i]));
      int x1 = max(0, min(SCREEN_WIDTH - 1, intersections[i + 1]));
      display.drawLine(x0, y, x1, y, WHITE);
    }
  }
}

void drawMouthBezier() {
  const int samples = 10; // interpolation resolution per segment

  for (int i = 0; i < 8; i++) {
    Point p0 = anchors[i];
    Point p1 = anchors[(i + 1) % 8];
    Point c0 = controls[i * 2 + 1];
    Point c1 = controls[((i + 1) % 8) * 2 + 0];

    Point prev = p0;

    // Approximate each cubic curve by many short straight lines
    for (int j = 1; j <= samples; j++) {
      float t = j / float(samples);
      Point p = cubicBezier(p0, c0, c1, p1, t);
      display.drawLine((int)round(prev.x), (int)round(prev.y),
                       (int)round(p.x), (int)round(p.y), WHITE);
      prev = p;
    }
  }
}

// Draws a filled polygon from the 8 anchor points (straight edges)
// using scanline fill with minimal memory and no extra libraries.
void fillMouthPoly() {
  int ymin = SCREEN_HEIGHT, ymax = 0;
  for (int i = 0; i < 8; i++) {
    int y = (int)round(anchors[i].y);
    ymin = min(ymin, y);
    ymax = max(ymax, y);
  }

  for (int y = max(0, ymin); y <= min(SCREEN_HEIGHT - 1, ymax); y++) {
    int intersections[16];
    int ni = 0;

    for (int i = 0; i < 8; i++) {
      Point a = anchors[i];
      Point b = anchors[(i + 1) % 8];
      if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
        float t = (y - a.y) / (b.y - a.y);
        float x = a.x + (b.x - a.x) * t;
        intersections[ni++] = (int)round(x);
      }
    }

    if (ni < 2) continue;
    for (int i = 0; i < ni-1; i++) {
      for (int j = i + 1; j < ni; j++) {
        if (intersections[j] < intersections[i]) {
          int tmp = intersections[i];
          intersections[i] = intersections[j];
          intersections[j] = tmp;
        }
      }
    }

    for (int i = 0; i + 1 < ni; i += 2) {
      int x0 = max(0, min(SCREEN_WIDTH - 1, intersections[i]));
      int x1 = max(0, min(SCREEN_WIDTH - 1, intersections[i + 1]));
      display.drawLine(x0, y, x1, y, WHITE);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow MPU6050 power-up time (needs ~30ms min, more is safer)
  Serial.println("Booting...");

  ledcSetup(buzzerChannel, 2000, buzzerResolution);
  ledcAttachPin(buzzerPin, buzzerChannel);
  playStartupBeep();
  setupPhoneTextServer();

  // OLED on Wire (SDA=21, SCL=22), MPU6050 on Wire1 (SDA=25, SCL=26)
  Wire.begin(21, 22);
  Wire.setClock(100000);
  scanI2CBus(Wire, "Wire (SDA=21 SCL=22)");

  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  Wire1.begin(25, 26);
  Wire1.setClock(100000);
  scanI2CBus(Wire1, "Wire1 (SDA=25 SCL=26)");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  // Try to find MPU6050 up to 5 times with delays (handles slow power-up)
  Serial.println("Looking for MPU6050...");
  for (int attempt = 1; attempt <= 5 && !mpuReady; attempt++) {
    Serial.print("Attempt ");
    Serial.print(attempt);
    Serial.print("/5... ");
    if (mpu.begin(0x68, &Wire1)) {
      mpuReady = true;
      Serial.println("found at 0x68");
    } else if (mpu.begin(0x69, &Wire1)) {
      mpuReady = true;
      Serial.println("found at 0x69");
    } else {
      Serial.println("not found");
      delay(300);
    }
  }

  if (!mpuReady) {
    Serial.println("MPU6050 FAILED on Wire1 (SDA=25 SCL=26).");
  }

  if (mpuReady) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  // Set initial filtered values to face center
  filteredX = faceCx;
  filteredY = faceCy;
}

void loop() {
  server.handleClient();
  serviceCharBeeps();

  // Step the animation phase
  tAnim += 0.06f;

  // Read gyroscope data from MPU6050
  if (mpuReady) {
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    gyroX = gyroEvent.gyro.x; // rad/s
    gyroY = gyroEvent.gyro.y; // rad/s
    gyroZ = gyroEvent.gyro.z; // rad/s
  } else {
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
  }

  // Print gyro values at 10 Hz for troubleshooting
  if (millis() - lastGyroPrintMs >= 200) {
    Serial.print("mpuReady=");
    Serial.print(mpuReady ? 1 : 0);
    Serial.print(" gyro(rad/s) x=");
    Serial.print(gyroX, 5);
    Serial.print(" y=");
    Serial.print(gyroY, 5);
    Serial.print(" z=");
    Serial.println(gyroZ, 5);
    lastGyroPrintMs = millis();
  }

  // Push face opposite of rotation direction (screen-space mapping)
  faceVelX += (-gyroX) * gyroPush;
  faceVelY += ( gyroY) * gyroPush;

  // Rotate face around center from gyro Z (toggle-able)
  if (rotateWithMotion) {
    faceAngVel += (-gyroZ) * gyroRotatePush;
    if (easeRotateBackToCenter) {
      faceAngVel += (-faceAngle) * angleSpringBack;
    }
    faceAngVel *= angleDamping;
    faceAngle += faceAngVel;
  } else {
    faceAngle = 0.0f;
    faceAngVel = 0.0f;
  }

  if (faceAngle < -maxFaceAngle) faceAngle = -maxFaceAngle;
  if (faceAngle >  maxFaceAngle) faceAngle =  maxFaceAngle;

  // Spring/damper so face naturally returns to center when rotation stops
  if (easeBackToCenter) {
    faceVelX += (-faceOffsetX) * springBack;
    faceVelY += (-faceOffsetY) * springBack;
    faceVelX *= damping;
    faceVelY *= damping;
    faceOffsetX += faceVelX;
    faceOffsetY += faceVelY;
  }

  // Limit how far the face can move from center
  if (faceOffsetX < -40) faceOffsetX = -40;
  if (faceOffsetX >  40) faceOffsetX =  40;
  if (faceOffsetY < -16) faceOffsetY = -16;
  if (faceOffsetY >  16) faceOffsetY =  16;

  // Target from gyro-driven offsets
  float targetX = 64 + faceOffsetX;
  float targetY = 40 + faceOffsetY;

  // Clamp target position to screen bounds
  if(targetX < 24) targetX = 24;
  if(targetX > 104) targetX = 104;
  if(targetY < 24) targetY = 24;
  if(targetY > 56) targetY = 56;

  // Lag filter: move filtered position toward target
  filteredX += (targetX - filteredX) * lagFactor;
  filteredY += (targetY - filteredY) * lagFactor;

  // Set face center to lagged position
  faceCx = filteredX;
  faceCy = filteredY;

  // Recompute the mouth and eye positions for this frame
  updateMouth();
  updateEyes();

  // Draw the scene
  display.clearDisplay();
  
  // Fill and draw left eye
  fillEyePoly(eyeAnchorsL);
  drawEyeBezier(eyeAnchorsL, eyeControlsL);
  
  // Fill and draw right eye
  fillEyePoly(eyeAnchorsR);
  drawEyeBezier(eyeAnchorsR, eyeControlsR);

  // Fill mouth shape first, then draw outline for crisp edge
  fillMouthPoly();
  drawMouthBezier();

  display.display();                                 // push to screen

  delay(30); // near 33Hz update rate
}


