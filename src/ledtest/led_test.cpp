// LED test, separate firmware (pio run -e testled -> dist/testled.bin): drives
// the RGB LED with a selectable color, intensity, mode and driver and shows
// the values sent, independent of the GPS logger.
//
// Keys: , /  color      ; .  intensity      F  mode (steady / blink / logger pulse)
//       R  refresh (send the same value again every n ms)
//       B  backlight brightness   H  backlight PWM frequency
//
// Cardputer ADV: the LED power switch is controlled by GPIO 38, the display
// backlight, which M5GFX dims with PWM (256 Hz): the LED loses its power with
// every PWM period below full brightness.
//       Space  LED off / on      D  driver (restarts)      G0  next color
#include <M5Cardputer.h>
#include <Preferences.h>

// Both drivers use the RMT peripheral on the same pin, so only one is used
// per boot; switching saves the choice and restarts.
enum Driver : uint8_t { DRV_CORE = 0, DRV_M5, DRV_COUNT };
static const char* const DRIVER_NAMES[DRV_COUNT] = {"neopixelWrite (core)", "M5.Led (M5Unified)"};

enum Mode : uint8_t { MODE_STEADY = 0, MODE_BLINK, MODE_PULSE, MODE_COUNT };
static const char* const MODE_NAMES[MODE_COUNT] = {"steady", "blink 1 Hz", "logger pulse 3 s"};

struct Color {
  const char* name;
  uint8_t r, g, b;
};
static const Color COLORS[] = {
  {"red", 255, 0, 0},     {"green", 0, 255, 0},   {"blue", 0, 0, 255},       {"white", 255, 255, 255},
  {"yellow", 255, 255, 0}, {"cyan", 0, 255, 255}, {"magenta", 255, 0, 255},
};
static const int COLOR_COUNT = sizeof(COLORS) / sizeof(COLORS[0]);

static M5Canvas canvas(&M5.Display);
static uint8_t driver = DRV_CORE;
static int pin = 21;
static bool m5Ok = false;  // M5.Led initialised (RMT channel created)
static int colorIdx = 0;
static int intensity = 160;  // 0-255, 160 = LED_MAX of the logger
static uint8_t mode = MODE_STEADY;
static bool ledOn = true;
static uint8_t sent[3] = {0, 0, 0};
static uint32_t writes = 0;
static uint32_t lastWriteMs = 0;
// The LED may lose its value: sending it again regularly shows whether that helps.
static const uint16_t REFRESHES[] = {0, 20, 100, 500};  // ms, 0 = only on changes
static const int REFRESH_COUNT = sizeof(REFRESHES) / sizeof(REFRESHES[0]);
static int refreshIdx = 0;
static const uint8_t BRIGHTS[] = {128, 192, 255, 64, 0};  // 128 = logger default
static const int BRIGHT_COUNT = sizeof(BRIGHTS) / sizeof(BRIGHTS[0]);
static int brightIdx = 0;
static const uint32_t BL_FREQS[] = {256, 25000};  // Hz, 256 = M5GFX default
static int freqIdx = 0;
static const uint8_t BL_PIN = 38, BL_CHANNEL = 7, BL_BITS = 9;  // as set up by M5GFX

static void writeLed(uint8_t r, uint8_t g, uint8_t b) {
  if (driver == DRV_CORE) neopixelWrite(pin, r, g, b);
  else M5.Led.setAllColor(r, g, b);
  sent[0] = r;
  sent[1] = g;
  sent[2] = b;
  writes++;
  lastWriteMs = millis();
}

// Intensity factor 0-255 for the current mode at this moment.
static int levelNow() {
  if (!ledOn) return 0;
  uint32_t t = millis();
  if (mode == MODE_BLINK) return (t / 500) % 2 ? 0 : intensity;
  if (mode == MODE_PULSE) {  // as in the logger: 1 s in, 1 s out (squared), then dark
    uint32_t p = t % 3000;
    if (p >= 2000) return 0;
    float f = (p < 1000 ? p : 2000 - p) / 1000.0f;
    return f * f * intensity;
  }
  return intensity;
}

static void updateLed() {
  static uint8_t last[3] = {1, 1, 1};  // differs from the first value: always written once
  const Color& c = COLORS[colorIdx];
  int lv = levelNow();
  uint8_t v[3] = {(uint8_t)(c.r * lv / 255), (uint8_t)(c.g * lv / 255), (uint8_t)(c.b * lv / 255)};
  uint16_t refresh = REFRESHES[refreshIdx];
  if (!memcmp(v, last, 3) && (!refresh || millis() - lastWriteMs < refresh)) return;
  memcpy(last, v, 3);
  writeLed(v[0], v[1], v[2]);
}

static void line(int y, const char* label, const String& value, uint16_t col) {
  canvas.setTextColor(TFT_DARKGREY);
  canvas.drawString(label, 3, y);
  canvas.setTextColor(col);
  canvas.drawString(value, 78, y);
}

static void draw() {
  canvas.fillSprite(TFT_BLACK);
  canvas.setFont(&fonts::Font2);
  canvas.fillRect(0, 0, 240, 18, 0x18E3);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("LED TEST", 3, 1);
  canvas.setTextColor(TFT_DARKGREY);
  canvas.setTextDatum(top_right);
  canvas.drawString(M5.getBoard() == m5::board_t::board_M5CardputerADV ? "Cardputer ADV" : "Cardputer", 237, 1);
  canvas.setTextDatum(top_left);

  const Color& c = COLORS[colorIdx];
  String drv = DRIVER_NAMES[driver];
  if (driver == DRV_M5) drv += m5Ok ? " ok" : " FAILED";
  line(20, "Driver", drv, driver == DRV_M5 && !m5Ok ? TFT_RED : TFT_CYAN);
  String pinText = String(pin);
  if (driver == DRV_M5) pinText += "  leds " + String(M5.Led.getCount());
  pinText += "   BL " + String(BRIGHTS[brightIdx]) + " @ " + String(BL_FREQS[freqIdx]) + " Hz";
  line(36, "Pin", pinText, TFT_CYAN);
  line(52, "Color", String(c.name) + "  (" + c.r + "," + c.g + "," + c.b + ")", TFT_YELLOW);
  line(68, "Intensity", String(intensity) + " / 255", TFT_YELLOW);
  String modeText = ledOn ? String(MODE_NAMES[mode]) : String("OFF");
  modeText += REFRESHES[refreshIdx] ? "  refr " + String(REFRESHES[refreshIdx]) + "ms" : String("  refr off");
  line(84, "Mode", modeText, ledOn ? TFT_YELLOW : TFT_RED);
  char buf[40];
  snprintf(buf, sizeof(buf), "R %3u  G %3u  B %3u", sent[0], sent[1], sent[2]);
  line(100, "Sent", buf, TFT_GREEN);
  canvas.fillRect(205, 98, 30, 18, canvas.color565(sent[0], sent[1], sent[2]));
  canvas.drawRect(205, 98, 30, 18, TFT_DARKGREY);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_DARKGREY);
  canvas.drawString(",/ col ;. int F mode R refr B/H backl", 3, 119);
  canvas.drawString("Spc off D driver G0 col writes " + String(writes), 3, 128);
  canvas.pushSprite(0, 0);
}

static void switchDriver() {
  Preferences prefs;
  prefs.begin("ledtest", false);
  prefs.putUChar("driver", (driver + 1) % DRV_COUNT);
  prefs.end();
  neopixelWrite(pin, 0, 0, 0);
  ESP.restart();
}

static void handleKey(char c) {
  if (c == ',') colorIdx = (colorIdx + COLOR_COUNT - 1) % COLOR_COUNT;
  else if (c == '/') colorIdx = (colorIdx + 1) % COLOR_COUNT;
  else if (c == ';') intensity = min(intensity + 16, 255);
  else if (c == '.') intensity = max(intensity - 16, 0);
  else if (c == 'f') mode = (mode + 1) % MODE_COUNT;
  else if (c == 'r') refreshIdx = (refreshIdx + 1) % REFRESH_COUNT;
  else if (c == 'b') {
    brightIdx = (brightIdx + 1) % BRIGHT_COUNT;
    M5.Display.setBrightness(BRIGHTS[brightIdx]);
  } else if (c == 'h') {
    freqIdx = (freqIdx + 1) % 2;
    ledcSetup(BL_CHANNEL, BL_FREQS[freqIdx], BL_BITS);
    ledcAttachPin(BL_PIN, BL_CHANNEL);
    M5.Display.setBrightness(BRIGHTS[brightIdx]);  // writes the duty again
  }
  else if (c == ' ') ledOn = !ledOn;
  else if (c == 'd') switchDriver();
}

void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, false);
  M5Cardputer.Keyboard.begin();
  M5.Display.setRotation(1);
  M5.Display.setBrightness(BRIGHTS[brightIdx]);
  canvas.setColorDepth(16);
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  Preferences prefs;
  prefs.begin("ledtest", true);
  driver = prefs.getUChar("driver", DRV_CORE) % DRV_COUNT;
  prefs.end();

  int p = M5.getPin(m5::pin_name_t::rgb_led);
  if (p >= 0 && p < GPIO_NUM_MAX) pin = p;  // 21 on both Cardputers
  if (driver == DRV_M5) {
    m5Ok = M5.Led.begin();
    M5.Led.setBrightness(255);  // default 63 scales values by (64/256)^2
  }
}

void loop() {
  M5.update();
  M5Cardputer.Keyboard.updateKeyList();
  M5Cardputer.Keyboard.updateKeysState();
  if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
    auto& ks = M5Cardputer.Keyboard.keysState();
    if (!ks.word.empty()) handleKey(tolower(ks.word[0]));
  }
  if (M5.BtnA.wasClicked()) colorIdx = (colorIdx + 1) % COLOR_COUNT;
  updateLed();
  static uint32_t lastDraw = 0;
  if (millis() - lastDraw > 100) {
    lastDraw = millis();
    draw();
  }
}
