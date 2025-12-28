#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <TFT_eSPI.h>
#include <Update.h>
#include <WiFiClientSecure.h>

// ---------------- WIFI ----------------
const char* ssid     = "POCOF4";
const char* password = "ryan1234";

// -----------------Global Variables----------------
unsigned long lastTimeQ = 0;
unsigned long timeQuery = 0;
unsigned long timeOTA = 0;
unsigned long lastTimeOTA = 0;
unsigned long now = 0;
int value = 0;

// ---------------- Trigger Defines ----------------
#define QUERY_INTERVAL  10000  // 10 seconds
#define OTA_INTERVAL    60000  // 60 seconds

// ---------------- OTA ----------------
const char* current_version = "1.0.3";
const char* version_url  = "https://raw.githubusercontent.com/sebry4n/robotika-kreatif-ota/main/version.txt";
const char* firmware_url = "https://raw.githubusercontent.com/sebry4n/robotika-kreatif-ota/main/firmware.bina=";

// ---------------- InfluxDB Cloud ----------------
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "T91ejO4cWzxMDSt7ene05gatEU5zErU_dVzmalp1PF3vYUzc2i46vlIjYyerJ6WWhkSv8H3Rz_ErGBzN9dbNOw=="
#define INFLUXDB_ORG "511bbff5ef8ad1d8"
#define INFLUXDB_BUCKET "desktopcompanion"

// ---------------- Query Flux ----------------
String fluxQuery =
  "from(bucket: \"desktopcompanion\")"
  " |> range(start: -20s)"
  " |> filter(fn: (r) => r._measurement == \"sentiment_value\")"
  " |> filter(fn: (r) => r._field == \"value\")"
  " |> last()";

WiFiClientSecure client;

// ---------------- TFT DISPLAY ----------------
TFT_eSPI tft = TFT_eSPI();           
TFT_eSprite screen = TFT_eSprite(&tft); // Main screen buffer
TFT_eSprite eye    = TFT_eSprite(&tft); // Sprite for a single eye

// --- TFT CONFIGURATION ---
// Set to 320x240 for Landscape mode
#define SCREEN_W 320
#define SCREEN_H 240 

// Cozmo Eye Settings
#define EYE_W 90       
#define EYE_H 110      
#define EYE_R 20       
#define EYE_GAP 85     

// Colors
#define BG_COLOR TFT_BLACK
#define EYE_COLOR TFT_CYAN 
// Define Sky Blue for tears if not default
#define TEAR_COLOR 0x6C1F // A nice lighter blue

// ---------------- MOTOR DEFINES ----------------
#define IN1_PIN  4
#define IN2_PIN  5
#define IN3_PIN  6
#define IN4_PIN  7

// ---------------- Motor Control Functions ----------------
void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void moveForward(int duration) {
  // Front Right Forward
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  // Back Right Forward
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  delay(duration);

  // Stop all motors
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void moveBackward(int duration) {
  // Front Right Backward
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  // Back Right Backward
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  delay(duration);

  // Stop all motors
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void turnLeft(int duration) {
  // Right motors forward
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);


  delay(duration);

  // Stop all motors
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void turnRight(int duration) {
  // Right motors backward
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  // Left motors forwar

  delay(duration);

  // Stop all motors
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

// ---------------- OTA FUNCTIONS ----------------
void performOTA() {
  HTTPClient https;
  https.begin(client, firmware_url);
  int httpCode = https.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = https.getSize();
    bool canBegin = Update.begin(contentLength);

    if (canBegin) {
      WiFiClient* stream = https.getStreamPtr();
      uint8_t buf[1024];
      size_t written = 0;

      Serial.printf("Starting OTA, size: %d bytes\n", contentLength);

      while (https.connected() && written < contentLength) {
        size_t len = stream->available();
        if (len) {
          if (len > sizeof(buf)) len = sizeof(buf);
          int c = stream->readBytes(buf, len);
          Update.write(buf, c);
          written += c;
          Serial.printf("Progress: %d/%d bytes\n", written, contentLength);
        }
        delay(1); // yield to avoid watchdog reset
      }

      if (Update.end() && Update.isFinished()) {
        Serial.println("Update successfully finished. Rebooting...");
        ESP.restart();
      } else {
        Serial.println("Update failed. Error #: " + String(Update.getError()));
      }
    } else {
      Serial.println("Not enough space to begin OTA");
    }
  } else {
    Serial.println("Failed to fetch firmware.bin, HTTP code: " + String(httpCode));
  }
  https.end();
}

void checkForOTA() {
  client.setInsecure();
  HTTPClient https;
  https.begin(client, version_url);
  int httpCode = https.GET();

  if (httpCode == HTTP_CODE_OK) {
    String new_version = https.getString();
    new_version.trim();
    Serial.println("Latest firmware version: " + new_version);

    if (new_version != current_version) {
      Serial.println("New firmware available! Updating...");
      performOTA();
    } else {
      Serial.println("Firmware is up to date.");
    }
  } else {
    Serial.println("Failed to fetch version.txt, HTTP code: " + String(httpCode));
  }
  https.end();
}

void getdata() {
  client.setCACert(InfluxDbCloud2CACert);
  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;

    String url = String(INFLUXDB_URL) + "/api/v2/query?org=" + INFLUXDB_ORG;
    http.begin(client, url);

    http.addHeader("Authorization", "Token " + String(INFLUXDB_TOKEN));
    http.addHeader("Content-Type", "application/vnd.flux");
    http.addHeader("Accept", "application/csv");

    Serial.println("\n--- Sending Query ---");
    Serial.println(fluxQuery);

    int httpCode = http.POST(fluxQuery);

    if (httpCode > 0) {
      Serial.printf("HTTP Code: %d\n", httpCode);

      if (httpCode == 200) {
        String payload = http.getString();
        Serial.println("=== RAW RESPONSE ===");
        Serial.println(payload);

        // ---------------- PARSE VALUE ----------------
        int headerEnd = payload.indexOf('\n');   // end of header
        if (headerEnd > 0) {
          String dataLine = payload.substring(headerEnd + 1);
          int nextLine = dataLine.indexOf('\n');
          if (nextLine > 0) {
            dataLine = dataLine.substring(0, nextLine);  // only 2nd line

            Serial.print("Data Line: ");
            Serial.println(dataLine);

            // split CSV to get _value (6th index)
            int col = 0;
            int start = 0;
            int comma = 0;
            String valueString = "0";

            while (col <= 6) {
              comma = dataLine.indexOf(',', start);
              if (comma == -1) comma = dataLine.length();

              if (col == 6) {  // _value column
                valueString = dataLine.substring(start, comma);
                break;
              }

              start = comma + 1;
              col++;
            }

            value = valueString.toInt();
            Serial.print("Parsed Value = ");
            Serial.println(value);
          }
        }
        // ----------------------------------------------

      }

    } else {
      Serial.printf("Error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }
}

// --- DRAWING HELPERS ---

// Standard Rectangular Eye
void drawEyeSprite(int h) {
  eye.fillSprite(BG_COLOR); 
  int spriteW = eye.width();
  int spriteH = eye.height();
  int x = (spriteW - EYE_W) / 2;
  int y = (spriteH - h) / 2;
  int r = (h < 20) ? 2 : EYE_R; 
  eye.fillRoundRect(x, y, EYE_W, h, r, EYE_COLOR);
}

// Happy Eye with "Cheek" Cutout (Bottom Curve)
void drawHappyEyeSprite(int h) {
  eye.fillSprite(BG_COLOR); 
  
  int spriteW = eye.width();
  int spriteH = eye.height();
  int x = (spriteW - EYE_W) / 2;
  int y = (spriteH - h) / 2;
  
  // 1. Draw the full rectangle first
  eye.fillRoundRect(x, y, EYE_W, h, EYE_R, EYE_COLOR);

  // 2. Draw a black circle at the bottom to create the "smile" curve
  // INCREASED offset to make the smile more pronounced (squintier)
  int cheekRadius = EYE_W + 10; 
  int offset = 35; // Pushed up higher for a bigger smile
  eye.fillCircle(spriteW / 2, y + h + (cheekRadius/2) - offset, cheekRadius, BG_COLOR);
}

// Sad Eye with "Heavy Eyelid" (Flat Top)
void drawSadEyeSprite(int h) {
  eye.fillSprite(BG_COLOR); 
  
  int spriteW = eye.width();
  int spriteH = eye.height();
  int x = (spriteW - EYE_W) / 2;
  int y = (spriteH - h) / 2;
  
  // 1. Draw the full rectangle first
  eye.fillRoundRect(x, y, EYE_W, h, EYE_R, EYE_COLOR);

  // 2. Draw a black rectangle at the top to flatten it (Heavy Eyelid)
  // This removes the top rounded corners and makes the eye look "low energy"
  int eyelidHeight = 25; 
  eye.fillRect(0, 0, spriteW, y + eyelidHeight, BG_COLOR);
}

// --- EXPRESSIONS ---

// 1. NEUTRAL
void animateNeutral(int duration) {
  unsigned long startTime = millis();
  unsigned long nextBlink = millis() + random(1000, 3000);
  bool isBlinking = false;
  unsigned long blinkEndTime = 0;

  while ((millis() - startTime) < duration) {
    unsigned long now = millis();

    if (now > nextBlink && !isBlinking) {
      isBlinking = true;
      blinkEndTime = now + 120; 
    }
    if (isBlinking && now > blinkEndTime) {
      isBlinking = false;
      nextBlink = now + random(2000, 5000); 
    }

    screen.fillSprite(BG_COLOR); 

    // --- IDLE ANIMATION (Breathing) ---
    // Increased Aggressiveness: Speed 180 (faster), Amplitude 6 (deeper)
    int bounceY = (sin(now / 180.0) * 6); 

    if (!isBlinking) {
      drawEyeSprite(EYE_H); 
      // Gentle tilt for neutral
      screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + bounceY);
      eye.pushRotated(&screen, -5, BG_COLOR); 
      screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + bounceY);
      eye.pushRotated(&screen, 5, BG_COLOR);
    } else {
      drawEyeSprite(10); // Blink
      screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + bounceY);
      eye.pushRotated(&screen, -5, BG_COLOR); 
      screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + bounceY);
      eye.pushRotated(&screen, 5, BG_COLOR);
    }
    screen.pushSprite(0, 0);
    delay(10); 
  }
}

// 2. HAPPY (NATURAL MIX)
// Alternates between "Excited" (Bounce/Smile) and "Content" (Slight Tilt/Normal Eyes)
void animateHappy(int duration) {
  unsigned long startTime = millis();
  unsigned long nextBlink = millis() + random(1000, 3000);
  bool isBlinking = false;
  unsigned long blinkEndTime = 0;

  // Logic to mix in neutral/content expressions
  bool isExcited = true; 
  unsigned long nextStateChange = millis() + 1000;

  while ((millis() - startTime) < duration) {
    unsigned long now = millis();

    // Blink Logic
    if (now > nextBlink && !isBlinking) {
      isBlinking = true;
      blinkEndTime = now + 120; 
    }
    if (isBlinking && now > blinkEndTime) {
      isBlinking = false;
      nextBlink = now + random(2000, 5000); 
    }

    // State Switching Logic: Toggle between Excited and Content
    if (now > nextStateChange) {
       isExcited = !isExcited;
       // Stay in new state for 1-2.5 seconds
       nextStateChange = now + random(1000, 2500);
    }

    screen.fillSprite(BG_COLOR); 

    // --- HAPPY ANIMATION (AGGRESSIVE) ---
    int bounceY = 0;
    if (isExcited) {
      // Hyperactive Bounce: Speed 100 (Very fast), Amplitude 12 (High jump)
       bounceY = (sin(now / 100.0) * 12); 
    } else {
      // Content Bounce: Speed 180, Amplitude 6 (Matches new neutral)
       bounceY = (sin(now / 180.0) * 6); 
    }

    if (!isBlinking) {
      if (isExcited) {
        // --- EXCITED STATE (Full Happiness) ---
        // Big Smile, 20 degree tilt, Bouncing
        drawHappyEyeSprite(EYE_H); 
        
        screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + bounceY);
        eye.pushRotated(&screen, -20, BG_COLOR); 
        
        screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + bounceY);
        eye.pushRotated(&screen, 20, BG_COLOR);
      } else {
        // --- CONTENT STATE (The Neutral Mix) ---
        // Normal Eyes (No Cheek), 8 degree tilt (Between Neutral's 5 and Happy's 20)
        drawEyeSprite(EYE_H); 
        
        screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + bounceY);
        eye.pushRotated(&screen, -8, BG_COLOR); 
        
        screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + bounceY);
        eye.pushRotated(&screen, 8, BG_COLOR);
      }
    } else {
      // Blink (Match rotation to current state so it doesn't snap)
      drawEyeSprite(10); 
      
      int currentTilt = isExcited ? 20 : 8;

      screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + bounceY);
      eye.pushRotated(&screen, -currentTilt, BG_COLOR); 
      screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + bounceY);
      eye.pushRotated(&screen, currentTilt, BG_COLOR);
    }
    
    screen.pushSprite(0, 0);
    delay(10);
  }
}

// 3. SAD (Updated for "Sadder" look)
void animateSad(int duration) {
  unsigned long startTime = millis();
  unsigned long nextBlink = millis() + random(1000, 3000);
  bool isBlinking = false;
  unsigned long blinkEndTime = 0;

  while ((millis() - startTime) < duration) {
    unsigned long now = millis();

    if (now > nextBlink && !isBlinking) {
      isBlinking = true;
      // Sad blinks are slower
      blinkEndTime = now + 200; 
    }
    if (isBlinking && now > blinkEndTime) {
      isBlinking = false;
      nextBlink = now + random(2000, 5000); 
    }

    screen.fillSprite(BG_COLOR); 

    // --- SAD ANIMATION (Deep Sigh) ---
    // Slow but Deep: Speed 280, Amplitude 8 (Heavy motion)
    int bounceY = (sin(now / 280.0) * 8); 

    if (!isBlinking) {
      // 1. Use the new Sad Sprite (Flat top)
      // 2. Reduce height slightly (Droopy/Tired look)
      drawSadEyeSprite(85); 
      
      // 3. Position Lower (+30 y offset) to look downcast, plus bounce
      // 4. Increase Tilt to 30 degrees for strong "worry" brow
      screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + 30 + bounceY); 
      eye.pushRotated(&screen, 30, BG_COLOR); 
      
      screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + 30 + bounceY);
      eye.pushRotated(&screen, -30, BG_COLOR);
    } else {
      drawEyeSprite(10); // Normal blink
      screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + 30 + bounceY);
      eye.pushRotated(&screen, 30, BG_COLOR); 
      screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + 30 + bounceY);
      eye.pushRotated(&screen, -30, BG_COLOR);
    }
    
    screen.pushSprite(0, 0);
    delay(10);
  }
}

// 4. CRYING (New Expression)
void animateCrying(int duration) {
  unsigned long startTime = millis();
  unsigned long nextBlink = millis() + random(1000, 3000);
  bool isBlinking = false;
  unsigned long blinkEndTime = 0;
  
  // Physics for tears
  // Start lower because eyes are lower
  float tearY = SCREEN_H / 2 + 60; 
  int tearOffset = 0; 

  while ((millis() - startTime) < duration) {
    unsigned long now = millis();

    if (now > nextBlink && !isBlinking) {
      isBlinking = true;
      blinkEndTime = now + 100; // Fast nervous blinks
    }
    if (isBlinking && now > blinkEndTime) {
      isBlinking = false;
      nextBlink = now + random(500, 1500); // Blinks often
    }

    screen.fillSprite(BG_COLOR); 

    // --- SOBBING ANIMATION ---
    // Combined Fast Jitter (shaking) and Slow Heave (breathing)
    int jitter = (sin(now / 60.0) * 4); // Fast shake
    int heave = (sin(now / 400.0) * 6); // Slow heave
    // Increased Offset to 50 (was 20) to make it look DOWN at the floor
    int totalY = jitter + heave + 50;   

    // --- TEAR PHYSICS ---
    tearY += 5; // Gravity
    if (tearY > SCREEN_H) {
      tearY = SCREEN_H / 2 + 60; // Reset to lower eye level
      tearOffset = random(-10, 10); // Randomize stream position slightly
    }

    // Draw Tears (Blue circles)
    // We draw them relative to where the eyes are
    screen.fillCircle(SCREEN_W / 2 - EYE_GAP + tearOffset, tearY, 5, TEAR_COLOR);
    screen.fillCircle(SCREEN_W / 2 + EYE_GAP - tearOffset, tearY, 5, TEAR_COLOR);

    if (!isBlinking) {
      // Use Sad sprite but smaller height (70) for squinting/pain
      drawSadEyeSprite(70); 
      
      // Extreme outward tilt (35 degrees)
      screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + totalY); 
      eye.pushRotated(&screen, 35, BG_COLOR); 
      
      screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + totalY); 
      eye.pushRotated(&screen, -35, BG_COLOR);
    } else {
      drawEyeSprite(10); 
      screen.setPivot(SCREEN_W / 2 - EYE_GAP, SCREEN_H / 2 + totalY); 
      eye.pushRotated(&screen, 35, BG_COLOR); 
      screen.setPivot(SCREEN_W / 2 + EYE_GAP, SCREEN_H / 2 + totalY); 
      eye.pushRotated(&screen, -35, BG_COLOR);
    }
    
    screen.pushSprite(0, 0);
    delay(10);
  }
}

// ---------------- Logic to choose expression based on sentiment value ----------------
void expressEmotion(int sentimentValue, int duration) {
  if (sentimentValue == 3) {
    animateHappy(duration);
    turnLeft(2000);
  }
  if (sentimentValue == 2) {
    animateNeutral(duration);
    moveForward(2000);
    turnLeft(5000);
    turnRight(4000);
  }
  if (sentimentValue == 1) {
    animateSad(duration);
    moveBackward(2000);
  }
  else {
    animateNeutral(duration);
  }
  value = 2;
}

// ----------------- SETUP & LOOP ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected.");

  // checkForOTA();
  // Load InfluxDB CA cert
  // client.setCACert(InfluxDbCloud2CACert);


  // TFT Setup
  tft.init();
  tft.setRotation(3); 
  
  // 1. Create the Main Screen Sprite
  screen.setColorDepth(8);
  screen.createSprite(SCREEN_W, SCREEN_H);

  // 2. Create the "Eye" Sprite
  eye.setColorDepth(8);
  eye.createSprite(EYE_W + 20, EYE_H + 20);
  eye.setPivot((EYE_W + 20) / 2, (EYE_H + 20) / 2);

  // Motor Pins Setup
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  stopMotors();
}

void loop() {
  now = millis();

  if (now - lastTimeQ>= QUERY_INTERVAL) {
    lastTimeQ = now;
    getdata();
  }

  // if (now - lastTimeOTA >= OTA_INTERVAL) { 
  //   lastTimeOTA = now;
  //   checkForOTA();
  // }
  
  expressEmotion(value, 10000); // Show expression for 8 seconds

}
