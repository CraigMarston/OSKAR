#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <painlessMesh.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <IRremote.hpp>
#include <Wire.h>
#include <ArduinoJson.h>
#include <chrono>
#include <map>
#include <vector>
#include <algorithm>
#include <WiFi.h>

// ============================================================================
// 1. COLOR DEFINITIONS
// ============================================================================
enum class DisplayColor : uint16_t {
    Black   = 0x0000,
    White   = 0xFFFF,
    Red     = 0xF800,
    Green   = 0x07E0,
    Blue    = 0x001F,
    Yellow  = 0xFFE0,
    Cyan    = 0x07FF,
    Magenta = 0xF81F,
    Orange  = 0xFD20
};

constexpr uint16_t toColor(DisplayColor c) {
    return static_cast<uint16_t>(c);
}

// ============================================================================
// 2. PIN DEFINITIONS & NICKNAMES
// ============================================================================
const char* FUTURAMA_NAMES[] = {
    "Fry",
    "Leela",
    "Bender",
    "Farnsworth",
    "Zoidberg",
    "Amy",
    "Hermes",
    "Zapp",
    "Kif",
    "Nibbler"
};
constexpr size_t NUM_FUTURAMA_NAMES = sizeof(FUTURAMA_NAMES) / sizeof(FUTURAMA_NAMES[0]);

namespace Pins {
    constexpr uint8_t Trigger       {2};
    constexpr uint8_t IR_Tx         {19};
    constexpr uint8_t Piezo         {12};
    constexpr uint8_t IR_Rx         {13};
    constexpr uint8_t Neopixels     {17};       // GPIO 17
    constexpr uint8_t BatteryAdc    {5};        

    // Rotary Encoder Pins
    constexpr uint8_t EncClk        {10};
    constexpr uint8_t EncDt         {7};
    constexpr uint8_t EncBtn        {8};

    // Dedicated Waveshare QMI8658 I2C Pins
    constexpr uint8_t TwiScl        {47}; 
    constexpr uint8_t TwiSda        {48}; 

    // LCD
    constexpr uint8_t LcdBl         {1};
    constexpr uint8_t LcdRst        {0};
    constexpr uint8_t LcdDc         {42};
    constexpr uint8_t LcdCs         {45};
    constexpr uint8_t LcdSclk       {39};
    constexpr uint8_t LcdMosi       {38};
    constexpr uint8_t LcdMiso       {40};
}

enum class PlayerStatus : uint8_t {
    SelectingName,
    PendingJoin,
    Accepted,
    Rejected
};

struct PlayerProfile {
    String baseNickname;
    String nickname;
    uint32_t nodeId;
    uint16_t irId;
    PlayerStatus status{PlayerStatus::SelectingName};
};

// ============================================================================
// 3. ROTARY ENCODER CONTROLLER
// ============================================================================
class RotaryEncoderController {
private:
    uint8_t clkPin_{Pins::EncClk};
    uint8_t dtPin_{Pins::EncDt};
    uint8_t btnPin_{Pins::EncBtn};

    uint8_t lastClk_{HIGH};
    bool btnState_{HIGH};
    bool lastBtnState_{HIGH};
    uint32_t lastDebounceMs_{0};

public:
    RotaryEncoderController() = default;

    void init() {
        pinMode(clkPin_, INPUT_PULLUP);
        pinMode(dtPin_, INPUT_PULLUP);
        pinMode(btnPin_, INPUT_PULLUP);
        lastClk_ = digitalRead(clkPin_);
    }

    int readRotation() {
        int dir = 0;
        uint8_t currentClk = digitalRead(clkPin_);
        
        if (currentClk != lastClk_ && currentClk == LOW) {
            if (digitalRead(dtPin_) != currentClk) {
                dir = 1;  
            } else {
                dir = -1; 
            }
        }
        lastClk_ = currentClk;
        return dir;
    }

    bool isButtonPressed() {
        bool reading = digitalRead(btnPin_);
        bool clicked = false;

        if (reading != lastBtnState_) {
            lastDebounceMs_ = millis();
        }

        if ((millis() - lastDebounceMs_) > 30) {
            if (reading != btnState_) {
                btnState_ = reading;
                if (btnState_ == LOW) {
                    clicked = true;
                }
            }
        }
        lastBtnState_ = reading;
        return clicked;
    }
};

// ============================================================================
// 4. BATTERY MONITOR
// ============================================================================
class BatteryMonitor {
private:
    uint8_t pin_{Pins::BatteryAdc};
    static constexpr float DividerRatio = 3.0f; 
    static constexpr float MinVoltage   = 3.0f; 
    static constexpr float MaxVoltage   = 4.2f; 

public:
    explicit BatteryMonitor(uint8_t pin = Pins::BatteryAdc) : pin_(pin) {}

    void init() {
        pinMode(pin_, INPUT);
        analogSetAttenuation(ADC_11db);
    }

    float getVoltage() const {
        uint32_t mv = analogReadMilliVolts(pin_);
        return (mv * DividerRatio) / 1000.0f;
    }

    uint8_t getPercentage() const {
        float v = getVoltage();
        if (v >= MaxVoltage) return 100;
        if (v <= MinVoltage) return 0;
        return static_cast<uint8_t>(((v - MinVoltage) / (MaxVoltage - MinVoltage)) * 100.0f);
    }
};

// ============================================================================
// 5. MOTION CONTROLLER
// ============================================================================
class MotionController {
private:
    uint8_t qmiAddress_{0x6B};
    bool initialized_{false};
    std::chrono::steady_clock::time_point lastReloadTime_{};
    static constexpr uint16_t ReloadCooldownMs = 1000;

    void writeRegister(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(qmiAddress_);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
        delay(5);
    }

public:
    MotionController() = default;

    void init() {
        uint8_t powerPins[] = {6, 15, 42};
        for (uint8_t p : powerPins) {
            pinMode(p, OUTPUT);
            digitalWrite(p, HIGH);
        }
        delay(50);

        Wire.begin(Pins::TwiSda, Pins::TwiScl);
        Wire.setClock(100000);

        writeRegister(0x60, 0xB0);
        delay(20);

        writeRegister(0x02, 0x60);
        writeRegister(0x03, 0x25);
        writeRegister(0x04, 0x25);
        writeRegister(0x08, 0x03);

        initialized_ = true;
        lastReloadTime_ = std::chrono::steady_clock::now();
    }

    bool detectTiltReload() {
        if (!initialized_) return false;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReloadTime_).count();
        if (elapsed < ReloadCooldownMs) return false;

        Wire.beginTransmission(qmiAddress_);
        Wire.write(0x35);
        if (Wire.endTransmission(false) != 0) return false;

        Wire.requestFrom(qmiAddress_, (uint8_t)6);
        if (Wire.available() < 6) return false;

        int16_t accelX = Wire.read() | (Wire.read() << 8);
        int16_t accelY = Wire.read() | (Wire.read() << 8);
        int16_t accelZ = Wire.read() | (Wire.read() << 8);

        if ((abs(accelX) + abs(accelY)) > 14000) { 
            lastReloadTime_ = now;
            return true;
        }
        return false;
    }
};

// ============================================================================
// 6. AUDIO CONTROLLER
// ============================================================================
class AudioController {
private:
    uint8_t pin_;
    bool isPlaying_{false};
    std::chrono::steady_clock::time_point noteStartTime_{};
    uint16_t currentNoteDuration_{0};

    struct Note {
        uint32_t frequency;
        uint16_t duration;
    };

    static constexpr size_t MaxNotes = 16;
    Note sequence_[MaxNotes];
    size_t sequenceLength_{0};
    size_t currentNoteIndex_{0};

public:
    explicit AudioController(uint8_t pin) : pin_(pin) {}

    void init() {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, LOW);
        ledcAttach(pin_, 2000, 8);
        ledcWrite(pin_, 0);
    }

    void playFireSound() {
        Note sfx[] = {{2800, 15}, {2400, 15}, {2000, 15}, {1600, 15}, {1200, 20}, {800, 20}};
        loadSequence(sfx, sizeof(sfx) / sizeof(Note));
    }

    void playHitSound() {
        Note sfx[] = {{400, 80}, {0, 20}, {250, 120}, {150, 150}};
        loadSequence(sfx, sizeof(sfx) / sizeof(Note));
    }

    void playReloadSound() {
        Note sfx[] = {{1319, 50}, {1568, 50}, {2637, 50}, {2093, 50}, {2349, 50}, {3136, 150}};
        loadSequence(sfx, sizeof(sfx) / sizeof(Note));
    }

    void playMenuSelectSound() {
        Note sfx[] = {{1046, 30}, {1318, 50}};
        loadSequence(sfx, sizeof(sfx) / sizeof(Note));
    }

    void update() {
        if (!isPlaying_) return;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - noteStartTime_).count();

        if (elapsed >= currentNoteDuration_) {
            currentNoteIndex_++;
            if (currentNoteIndex_ < sequenceLength_) {
                playCurrentNote();
            } else {
                stop();
            }
        }
    }

    void stop() {
        isPlaying_ = false;
        ledcWrite(pin_, 0);
    }

private:
    void loadSequence(const Note* notes, size_t count) {
        count = std::min(count, MaxNotes);
        for (size_t i = 0; i < count; ++i) sequence_[i] = notes[i];
        sequenceLength_ = count;
        currentNoteIndex_ = 0;
        playCurrentNote();
    }

    void playCurrentNote() {
        Note current = sequence_[currentNoteIndex_];
        currentNoteDuration_ = current.duration;
        noteStartTime_ = std::chrono::steady_clock::now();
        isPlaying_ = true;

        if (current.frequency > 0) {
            ledcChangeFrequency(pin_, current.frequency, 8);
            ledcWrite(pin_, 127);
        } else {
            ledcWrite(pin_, 0);
        }
    }
};

// ============================================================================
// 7. DISPLAY MANAGER (240 x 320 PIXELS)
// ============================================================================
class DisplayManager {
private:
    Arduino_ESP32SPI* bus_ {nullptr};
    Arduino_ST7789*   gfx_ {nullptr};
    uint8_t lastRenderedBattPct_{255};
    int8_t lastRenderedRssi_{0};

    void drawHeart(int16_t x, int16_t y, bool filled) {
        if (!gfx_) return;
        if (filled) {
            gfx_->fillCircle(x + 4, y + 4, 4, toColor(DisplayColor::Red));
            gfx_->fillCircle(x + 11, y + 4, 4, toColor(DisplayColor::Red));
            gfx_->fillTriangle(x, y + 6, x + 15, y + 6, x + 7, y + 14, toColor(DisplayColor::Red));
        } else {
            uint16_t darkGrey = 0x4208;
            gfx_->drawCircle(x + 4, y + 4, 4, darkGrey);
            gfx_->drawCircle(x + 11, y + 4, 4, darkGrey);
            gfx_->drawTriangle(x, y + 6, x + 15, y + 6, x + 7, y + 14, darkGrey);
        }
    }

    void drawBulletIcon(int16_t x, int16_t y) {
        if (!gfx_) return;
        gfx_->fillRect(x + 2, y + 6, 6, 10, toColor(DisplayColor::Yellow));
        gfx_->fillTriangle(x + 2, y + 6, x + 7, y + 6, x + 4, y + 1, toColor(DisplayColor::Red));
    }

    void drawBatteryIcon(int16_t x, int16_t y, uint8_t percentage) {
        if (!gfx_) return;
        gfx_->fillRect(x - 30, y, 62, 16, toColor(DisplayColor::Black));
        uint16_t fillColor = (percentage > 20) ? toColor(DisplayColor::Green) : toColor(DisplayColor::Red);

        gfx_->drawRect(x, y + 2, 22, 12, toColor(DisplayColor::White));
        gfx_->fillRect(x + 22, y + 5, 2, 6, toColor(DisplayColor::White));

        uint8_t fillWidth = (18 * percentage) / 100;
        if (fillWidth > 0) gfx_->fillRect(x + 2, y + 4, fillWidth, 8, fillColor);

        gfx_->setCursor(x - 28, y + 4);
        gfx_->setTextColor(fillColor);
        gfx_->setTextSize(1);
        gfx_->setFont(nullptr);
        gfx_->printf("%3u%%", percentage);
    }

public:
    DisplayManager() = default;

    void init() {
        bus_ = new Arduino_ESP32SPI(Pins::LcdDc, Pins::LcdCs, Pins::LcdSclk, Pins::LcdMosi, Pins::LcdMiso);
        gfx_ = new Arduino_ST7789(bus_, Pins::LcdRst, 2, true, 240, 320);

        if (!gfx_->begin()) return;

        ledcAttach(Pins::LcdBl, 5000, 10);
        setBrightness(80);

        gfx_->fillScreen(toColor(DisplayColor::Black));
        gfx_->setCursor(10, 15);
        gfx_->setTextColor(toColor(DisplayColor::White));
        gfx_->setTextSize(2);
        gfx_->println("LASER TAG");
        gfx_->drawFastHLine(10, 40, 220, toColor(DisplayColor::White));
    }

    void setBrightness(uint8_t percentage) {
        uint32_t duty = (1023 * std::min<uint8_t>(percentage, 100)) / 100;
        ledcWrite(Pins::LcdBl, duty);
    }

    void updateBattery(uint8_t percentage) {
        if (percentage == lastRenderedBattPct_) return;
        lastRenderedBattPct_ = percentage;
        drawBatteryIcon(200, 15, percentage);
    }

    void drawSignalIcon(int16_t x, int16_t y, int8_t rssi, bool forceRedraw = false) {
        if (!gfx_) return;
        if (!forceRedraw && rssi == lastRenderedRssi_) return;
        lastRenderedRssi_ = rssi;

        gfx_->fillRect(x - 30, y, 50, 14, toColor(DisplayColor::Black));

        uint8_t bars = (rssi > -65) ? 4 : (rssi > -75) ? 3 : (rssi > -85) ? 2 : (rssi > -95) ? 1 : 0;
        uint16_t activeColor = (bars <= 1) ? toColor(DisplayColor::Red) : toColor(DisplayColor::Green);

        for (uint8_t i = 0; i < 4; ++i) {
            int16_t barHeight = (i + 1) * 3;
            gfx_->fillRect(x + (i * 4), y + (12 - barHeight), 3, barHeight, (i < bars) ? activeColor : 0x4208);
        }

        gfx_->setTextSize(1);
        gfx_->setFont(nullptr);
        gfx_->setTextColor(activeColor);
        gfx_->setCursor(x - 30, y + 3);

        if (rssi <= -127 || rssi == 0) {
            gfx_->print("N/A");
        } else {
            gfx_->printf("%d", rssi);
        }
    }

    void renderNamePicker(size_t currentIndex) {
        if (!gfx_) return;
        gfx_->fillRect(10, 45, 220, 180, toColor(DisplayColor::Black));

        gfx_->setCursor(15, 50);
        gfx_->setTextColor(toColor(DisplayColor::Yellow));
        gfx_->setTextSize(2);
        gfx_->println("SELECT NAME");

        gfx_->setCursor(15, 75);
        gfx_->setTextColor(toColor(DisplayColor::White));
        gfx_->setTextSize(1);
        gfx_->println("Turn knob, press to pick:");

        int16_t yPos = 100;
        size_t startIdx = (currentIndex >= 3) ? currentIndex - 3 : 0;
        size_t endIdx = std::min(startIdx + 5, NUM_FUTURAMA_NAMES);

        for (size_t i = startIdx; i < endIdx; ++i) {
            if (i == currentIndex) {
                gfx_->fillRect(15, yPos - 2, 210, 18, toColor(DisplayColor::Cyan));
                gfx_->setTextColor(toColor(DisplayColor::Black));
                gfx_->setCursor(20, yPos);
                gfx_->printf("> %s\n", FUTURAMA_NAMES[i]);
            } else {
                gfx_->setTextColor(toColor(DisplayColor::White));
                gfx_->setCursor(20, yPos);
                gfx_->printf("  %s\n", FUTURAMA_NAMES[i]);
            }
            yPos += 20;
        }
    }

    void renderJoinRequestScreen(String myNickname) {
        if (!gfx_) return;
        gfx_->fillRect(10, 45, 220, 180, toColor(DisplayColor::Black));
        
        gfx_->setCursor(15, 55);
        gfx_->setTextColor(toColor(DisplayColor::Yellow));
        gfx_->setTextSize(2);
        gfx_->println("WELCOME!");

        gfx_->setCursor(15, 85);
        gfx_->setTextColor(toColor(DisplayColor::White));
        gfx_->setTextSize(1);
        gfx_->printf("Name: %s\n", myNickname.c_str());

        gfx_->setCursor(15, 110);
        gfx_->setTextColor(toColor(DisplayColor::Cyan));
        gfx_->println("Press knob to send request:");

        gfx_->drawRect(35, 135, 170, 40, toColor(DisplayColor::Green));
        gfx_->fillRect(37, 137, 166, 36, toColor(DisplayColor::Green));

        gfx_->setCursor(55, 148);
        gfx_->setTextColor(toColor(DisplayColor::Black));
        gfx_->setTextSize(2);
        gfx_->println("JOIN GAME?");
    }

    void renderAcceptPlayerPrompt(String applicantName, bool acceptSelected) {
        if (!gfx_) return;
        gfx_->fillRect(10, 45, 220, 180, toColor(DisplayColor::Black));

        gfx_->setCursor(15, 50);
        gfx_->setTextColor(toColor(DisplayColor::Orange));
        gfx_->setTextSize(1);
        gfx_->println("NEW PLAYER REQUEST!");

        gfx_->setCursor(15, 70);
        gfx_->setTextColor(toColor(DisplayColor::White));
        gfx_->setTextSize(2);
        gfx_->println(applicantName);

        // ACCEPT Button
        uint16_t acceptBg = acceptSelected ? toColor(DisplayColor::Green) : toColor(DisplayColor::Black);
        gfx_->drawRect(20, 110, 90, 35, toColor(DisplayColor::Green));
        gfx_->fillRect(21, 111, 88, 33, acceptBg);
        gfx_->setCursor(30, 122);
        gfx_->setTextColor(acceptSelected ? toColor(DisplayColor::Black) : toColor(DisplayColor::Green));
        gfx_->setTextSize(1);
        gfx_->println("ACCEPT");

        // REJECT Button
        uint16_t rejectBg = !acceptSelected ? toColor(DisplayColor::Red) : toColor(DisplayColor::Black);
        gfx_->drawRect(130, 110, 90, 35, toColor(DisplayColor::Red));
        gfx_->fillRect(131, 111, 88, 33, rejectBg);
        gfx_->setCursor(145, 122);
        gfx_->setTextColor(!acceptSelected ? toColor(DisplayColor::White) : toColor(DisplayColor::Red));
        gfx_->setTextSize(1);
        gfx_->println("REJECT");
    }

    void renderLobby(String nickname, size_t peerCount, const std::map<uint32_t, PlayerProfile>& registry) {
        if (!gfx_) return;

        gfx_->fillRect(10, 45, 220, 165, toColor(DisplayColor::Black));
        gfx_->setTextSize(1);
        gfx_->setFont(nullptr);
        
        gfx_->setCursor(10, 50);
        gfx_->setTextColor(toColor(DisplayColor::Yellow));
        gfx_->printf("You: %s\n", nickname.c_str());

        gfx_->setCursor(10, 65);
        gfx_->setTextColor(toColor(DisplayColor::Cyan));
        gfx_->printf("Active Peers: %zu\n", peerCount);
        
        gfx_->drawFastHLine(10, 80, 220, 0x4208);

        gfx_->setCursor(10, 88);
        gfx_->setTextColor(toColor(DisplayColor::White));
        gfx_->println("LOBBY PLAYERS:");

        int16_t yPos = 102;
        for (const auto& [id, profile] : registry) {
            if (yPos > 200) break;
            if (profile.status != PlayerStatus::Accepted) continue;

            gfx_->setCursor(15, yPos);
            gfx_->setTextColor(toColor(DisplayColor::Green));
            gfx_->printf("- %s [0x%04X]\n", profile.nickname.c_str(), profile.irId);
            yPos += 14;
        }
    }

    void updateHUD(uint8_t health, uint8_t ammo) {
        if (!gfx_) return;

        // Clear Bottom HUD Area (from Y=210 down to 320)
        gfx_->fillRect(0, 210, 240, 110, toColor(DisplayColor::Black));
        gfx_->drawFastHLine(10, 210, 220, 0x4208);

        // AMMO AREA
        gfx_->setCursor(10, 220);
        gfx_->setTextColor(toColor(DisplayColor::White));
        gfx_->setTextSize(1);
        gfx_->println("AMMO");

        drawBulletIcon(10, 235);

        gfx_->setCursor(25, 233);
        gfx_->setTextColor((ammo > 5) ? toColor(DisplayColor::Green) : (ammo > 0) ? toColor(DisplayColor::Yellow) : toColor(DisplayColor::Red));
        gfx_->setTextSize(3);
        gfx_->printf("%02u", ammo);

        // HEALTH AREA (Shifted to Very Bottom Y=270..310, 6 Hearts)
        gfx_->drawFastHLine(10, 268, 220, 0x4208);
        
        gfx_->setCursor(10, 275);
        gfx_->setTextColor(toColor(DisplayColor::White));
        gfx_->setTextSize(1);
        gfx_->println("HEALTH");

        for (uint8_t i = 0; i < 6; ++i) {
            drawHeart(10 + (i * 20), 292, (i < health));
        }
    }
};

// ============================================================================
// 8. NEOPIXEL CONTROLLER
// ============================================================================
class NeoPixelController {
private:
    Adafruit_NeoPixel strip_{25, Pins::Neopixels, NEO_GRB + NEO_KHZ800};
    
    static constexpr uint16_t SacrificialIndex = 0;
    static constexpr uint16_t HitRingStart     = 1;
    static constexpr uint16_t HitRingCount     = 12;
    static constexpr uint16_t MuzzleRingStart  = 13;
    static constexpr uint16_t MuzzleRingCount  = 12;

    enum class EffectState { None, MuzzleFlash, HitFlash };
    EffectState activeEffect_{EffectState::None};

    std::chrono::steady_clock::time_point effectStartTime_{};
    uint16_t effectDurationMs_{0};

    uint8_t currentHealth_{6};

public:
    NeoPixelController() = default;

    void init() {
        strip_.begin();
        strip_.setBrightness(60);
        clearAll();
        applyIdleHealthColor();
    }

    void clearAll() {
        for (uint16_t i = 0; i < 25; ++i) {
            strip_.setPixelColor(i, 0);
        }
        strip_.show();
    }

    void setHitRingColor(uint8_t r, uint8_t g, uint8_t b) {
        for (uint16_t i = HitRingStart; i < (HitRingStart + HitRingCount); ++i) {
            strip_.setPixelColor(i, strip_.Color(r, g, b));
        }
        strip_.setPixelColor(SacrificialIndex, 0);
        strip_.show();
    }

    void setMuzzleRingColor(uint8_t r, uint8_t g, uint8_t b) {
        for (uint16_t i = MuzzleRingStart; i < (MuzzleRingStart + MuzzleRingCount); ++i) {
            strip_.setPixelColor(i, strip_.Color(r, g, b));
        }
        strip_.setPixelColor(SacrificialIndex, 0);
        strip_.show();
    }

    void triggerMuzzleFlash(uint16_t durationMs = 40) {
        for (uint16_t i = MuzzleRingStart; i < (MuzzleRingStart + MuzzleRingCount); ++i) {
            uint8_t spark = random(0, 3);
            if (spark == 0)      strip_.setPixelColor(i, strip_.Color(255, 255, 255)); 
            else if (spark == 1) strip_.setPixelColor(i, strip_.Color(255, 140, 0));   
            else                 strip_.setPixelColor(i, strip_.Color(255, 215, 0));   
        }
        
        strip_.setPixelColor(SacrificialIndex, 0);
        strip_.show();

        effectStartTime_ = std::chrono::steady_clock::now();
        effectDurationMs_ = durationMs;
        activeEffect_ = EffectState::MuzzleFlash;
    }

    void triggerHitEffect(uint8_t remainingHealth, uint16_t durationMs = 350) {
        currentHealth_ = remainingHealth;
        setHitRingColor(255, 0, 0);

        effectStartTime_ = std::chrono::steady_clock::now();
        effectDurationMs_ = durationMs;
        activeEffect_ = EffectState::HitFlash;
    }

    void updateHealthStatus(uint8_t health) {
        currentHealth_ = health;
        if (activeEffect_ == EffectState::None) {
            applyIdleHealthColor();
        }
    }

    void update() {
        if (activeEffect_ == EffectState::None) return;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - effectStartTime_).count();

        if (elapsed >= effectDurationMs_) {
            if (activeEffect_ == EffectState::MuzzleFlash) {
                setMuzzleRingColor(0, 0, 0);
            }
            activeEffect_ = EffectState::None;
            applyIdleHealthColor();
        }
    }

private:
    void applyIdleHealthColor() {
        uint16_t activeLeds = currentHealth_ * 2;
        if (activeLeds > HitRingCount) activeLeds = HitRingCount;

        for (uint16_t i = 0; i < HitRingCount; ++i) {
            uint16_t pixelIdx = HitRingStart + i;
            if (i < activeLeds) {
                strip_.setPixelColor(pixelIdx, strip_.Color(0, 40, 0)); 
            } else {
                strip_.setPixelColor(pixelIdx, 0);
            }
        }

        if (currentHealth_ == 0) {
            strip_.setPixelColor(HitRingStart, strip_.Color(20, 0, 0));
        }

        strip_.setPixelColor(SacrificialIndex, 0);
        strip_.show();
    }
};

// ============================================================================
// 9. INFRARED CONTROLLER
// ============================================================================
class IRController {
private:
    uint16_t myIrId_{0};
    static constexpr uint8_t MagicHeader {0xAB};
    static constexpr uint8_t ExpectedBits{32};

    uint8_t calculateChecksum(uint16_t playerId) const {
        return (MagicHeader ^ (playerId & 0xFF) ^ ((playerId >> 8) & 0xFF));
    }

public:
    IRController() = default;

    static uint16_t generateIrIdFromNodeId(uint32_t nodeId) {
        return static_cast<uint16_t>((nodeId >> 16) & 0xFFFF) ^ static_cast<uint16_t>(nodeId & 0xFFFF);
    }

    void init(uint32_t uniqueNodeId) {
        myIrId_ = generateIrIdFromNodeId(uniqueNodeId);
        IrReceiver.begin(Pins::IR_Rx, DISABLE_LED_FEEDBACK);
        IrSender.begin(Pins::IR_Tx, DISABLE_LED_FEEDBACK);
    }

    void fireShot() {
        IrReceiver.stop();
        uint32_t packet = ((uint32_t)MagicHeader << 24) | ((uint32_t)myIrId_ << 8) | calculateChecksum(myIrId_);
        IrSender.sendPulseDistanceWidth(38, 9000, 4500, 560, 1690, 560, 560, packet, ExpectedBits, PROTOCOL_IS_LSB_FIRST, 0, 0);
        IrReceiver.start();
    }

    bool checkForHit(uint16_t& attackerIrIdOut) {
        if (IrReceiver.decode()) {
            if (IrReceiver.decodedIRData.numberOfBits == ExpectedBits) {
                uint32_t packet = IrReceiver.decodedIRData.decodedRawData;
                uint8_t header = (packet >> 24) & 0xFF;
                uint16_t senderId = (packet >> 8) & 0xFFFF;
                uint8_t checksum = packet & 0xFF;

                if (header == MagicHeader && checksum == calculateChecksum(senderId) && senderId != myIrId_) {
                    attackerIrIdOut = senderId;
                    IrReceiver.resume();
                    return true;
                }
            }
            IrReceiver.resume();
        }
        return false;
    }
};

// ============================================================================
// 10. MESH NETWORK WITH JOIN / ACCEPT SYSTEM
// ============================================================================
class MeshController {
private:
    painlessMesh mesh_;
    
    String baseNickname_  {"Player"};
    String localNickname_ {"Player"};
    uint32_t localNodeId_ {0};
    uint16_t localIrId_   {0};

    PlayerStatus myStatus_ {PlayerStatus::SelectingName};

    std::map<uint32_t, PlayerProfile> registry_;
    volatile bool registryUpdated_ {false};
    uint32_t lastBroadcastMs_ {0};

    enum MessageType : uint8_t {
        MSG_NAME_ANNOUNCE = 1,
        MSG_JOIN_DECISION = 2
    };

    void resolveNames() {
        std::vector<uint32_t> activeNodes;
        activeNodes.push_back(localNodeId_);

        for (uint32_t id : mesh_.getNodeList()) {
            activeNodes.push_back(id);
        }

        std::sort(activeNodes.begin(), activeNodes.end());

        std::map<String, size_t> baseCounts;
        for (uint32_t id : activeNodes) {
            if (registry_.find(id) != registry_.end()) {
                baseCounts[registry_[id].baseNickname]++;
            }
        }

        std::map<String, size_t> runningRank;
        for (uint32_t id : activeNodes) {
            String base = registry_[id].baseNickname;
            if (baseCounts[base] > 1) {
                runningRank[base]++;
                registry_[id].nickname = (runningRank[base] == 1) ? base : base + " (" + String(runningRank[base]) + ")";
            } else {
                registry_[id].nickname = base;
            }

            if (id == localNodeId_) {
                localNickname_ = registry_[id].nickname;
            }
        }
    }

    void handleReceivedMessage(uint32_t from, String &msg) {
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, msg)) return;

        uint8_t type = doc["type"];
        uint32_t nodeId = doc["nodeId"];

        if (type == MSG_NAME_ANNOUNCE) {
            String remoteBaseName = doc["nickname"] | "Player";
            uint16_t irId = doc["irId"];
            PlayerStatus status = static_cast<PlayerStatus>((uint8_t)doc["status"]);

            registry_[nodeId] = {remoteBaseName, remoteBaseName, nodeId, irId, status};
            resolveNames();
            registryUpdated_ = true;
        } 
        else if (type == MSG_JOIN_DECISION) {
            uint32_t targetNode = doc["targetNode"];
            bool approved = doc["approved"];

            if (targetNode == localNodeId_) {
                myStatus_ = approved ? PlayerStatus::Accepted : PlayerStatus::Rejected;
                registry_[localNodeId_].status = myStatus_;
            } else if (registry_.find(targetNode) != registry_.end()) {
                registry_[targetNode].status = approved ? PlayerStatus::Accepted : PlayerStatus::Rejected;
            }
            registryUpdated_ = true;
        }
    }

public:
    MeshController() = default;

    void init() {
        mesh_.setDebugMsgTypes(ERROR);
        mesh_.init("laserTagMesh", "meshPassword123", 5555, WIFI_AP_STA, 6);

        localNodeId_ = mesh_.getNodeId(); 
        localIrId_ = IRController::generateIrIdFromNodeId(localNodeId_);

        registry_[localNodeId_] = {baseNickname_, baseNickname_, localNodeId_, localIrId_, myStatus_};

        mesh_.onReceive([this](uint32_t from, String &msg) {
            this->handleReceivedMessage(from, msg);
        });

        mesh_.onNewConnection([this](uint32_t nodeId) {
            this->broadcastIdentity();
        });
    }

    int8_t getRssi() {
        int8_t staRssi = WiFi.RSSI();
        if (staRssi < 0 && staRssi > -120) {
            return staRssi;
        }

        wifi_sta_list_t sta_list;
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK && sta_list.num > 0) {
            int32_t totalRssi = 0;
            for (int i = 0; i < sta_list.num; i++) {
                totalRssi += sta_list.sta[i].rssi;
            }
            return static_cast<int8_t>(totalRssi / sta_list.num);
        }

        return -127;
    }

    bool hasAcceptedHostOnMesh() {
        for (const auto& [id, profile] : registry_) {
            if (id != localNodeId_ && profile.status == PlayerStatus::Accepted) {
                return true;
            }
        }
        return false;
    }

    void setNickname(String newName) {
        baseNickname_ = newName;
        localNickname_ = newName;
        registry_[localNodeId_].baseNickname = newName;
        registry_[localNodeId_].nickname = newName;
        
        // Auto-promote to Accepted Host if no accepted host exists yet
        if (!hasAcceptedHostOnMesh()) {
            myStatus_ = PlayerStatus::Accepted;
        } else {
            myStatus_ = PlayerStatus::PendingJoin;
        }

        registry_[localNodeId_].status = myStatus_;
        broadcastIdentity();
    }

    void setLocalStatus(PlayerStatus st) {
        myStatus_ = st;
        registry_[localNodeId_].status = st;
        broadcastIdentity();
    }

    void sendJoinDecision(uint32_t targetNodeId, bool approve) {
        StaticJsonDocument<256> doc;
        doc["type"]       = MSG_JOIN_DECISION;
        doc["targetNode"] = targetNodeId;
        doc["approved"]   = approve;

        String output;
        serializeJson(doc, output);
        mesh_.sendBroadcast(output);

        if (registry_.find(targetNodeId) != registry_.end()) {
            registry_[targetNodeId].status = approve ? PlayerStatus::Accepted : PlayerStatus::Rejected;
            registryUpdated_ = true;
        }
    }

    void broadcastIdentity() {
        StaticJsonDocument<256> doc;
        doc["type"]     = MSG_NAME_ANNOUNCE;
        doc["nodeId"]   = localNodeId_;
        doc["irId"]     = localIrId_;
        doc["nickname"] = baseNickname_;
        doc["status"]   = static_cast<uint8_t>(myStatus_);

        String output;
        serializeJson(doc, output);
        mesh_.sendBroadcast(output);
    }

    void update() {
        mesh_.update();
        if (millis() - lastBroadcastMs_ > 1500) {
            lastBroadcastMs_ = millis();
            broadcastIdentity();
        }
    }

    bool hasRegistryUpdated() {
        if (registryUpdated_) {
            registryUpdated_ = false;
            return true;
        }
        return false;
    }

    uint32_t getPendingApplicantNodeId() {
        for (const auto& [id, profile] : registry_) {
            if (profile.status == PlayerStatus::PendingJoin && id != localNodeId_) {
                return id;
            }
        }
        return 0;
    }

    String getNicknameForNodeId(uint32_t nodeId) {
        if (registry_.find(nodeId) != registry_.end()) {
            return registry_[nodeId].nickname;
        }
        return "Unknown";
    }

    PlayerStatus getLocalStatus() const { return myStatus_; }
    String getLocalNickname() const { return localNickname_; }
    uint32_t getNodeId() const { return localNodeId_; }
    size_t getPeerCount() { return mesh_.getNodeList().size(); }
    const std::map<uint32_t, PlayerProfile>& getRegistry() const { return registry_; }
};

// ============================================================================
// 11. MAIN APPLICATION
// ============================================================================
class LaserTagApp {
private:
    DisplayManager display_;
    MeshController mesh_;
    IRController ir_;
    NeoPixelController leds_;
    AudioController audio_{Pins::Piezo};
    MotionController motion_;
    BatteryMonitor battery_;
    RotaryEncoderController encoder_;
    
    std::chrono::steady_clock::time_point lastShotTime_ {};
    std::chrono::steady_clock::time_point lastBatteryCheck_ {};
    
    uint8_t health_ {6};
    uint8_t ammo_ {30};
    bool lastTriggerState_ {HIGH};

    // UI Menu State
    size_t namePickerIndex_{0};
    bool acceptChoiceSelected_{true}; 
    uint32_t currentPromptApplicant_{0};
    bool joinScreenDrawn_{false};

    static constexpr uint16_t FireRateLimitMs = 200;

public:
    void setup() {
        Serial.begin(115200);
        delay(500);

        pinMode(Pins::Trigger, INPUT_PULLUP);

        encoder_.init();
        battery_.init();
        audio_.init();
        display_.init();
        leds_.init();

        mesh_.init(); 
        ir_.init(mesh_.getNodeId());
        motion_.init();

        display_.updateBattery(battery_.getPercentage());
        
        // RSSI shifted +2px right to X=154
        display_.drawSignalIcon(154, 16, mesh_.getRssi(), true);
        display_.renderNamePicker(namePickerIndex_);

        auto now = std::chrono::steady_clock::now();
        lastShotTime_ = now;
        lastBatteryCheck_ = now;
    }

    void loop() {
        mesh_.update();
        audio_.update();
        leds_.update();

        PlayerStatus status = mesh_.getLocalStatus();

        if (status == PlayerStatus::SelectingName) {
            handleNameSelectionMenu();
        } else if (status == PlayerStatus::PendingJoin) {
            handleJoinMenu();
        } else if (status == PlayerStatus::Accepted) {
            uint32_t applicantId = mesh_.getPendingApplicantNodeId();
            if (applicantId != 0) {
                handleHostApprovalMenu(applicantId);
            } else {
                if (currentPromptApplicant_ != 0) {
                    currentPromptApplicant_ = 0;
                    display_.renderLobby(mesh_.getLocalNickname(), mesh_.getPeerCount(), mesh_.getRegistry());
                    display_.updateHUD(health_, ammo_);
                }
                handleInGameLoop();
            }
        }

        if (mesh_.hasRegistryUpdated() && mesh_.getLocalStatus() == PlayerStatus::Accepted && currentPromptApplicant_ == 0) {
            display_.renderLobby(mesh_.getLocalNickname(), mesh_.getPeerCount(), mesh_.getRegistry());
            display_.updateHUD(health_, ammo_);
        }

        handleBatteryAndSignalMonitoring();
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

private:
    void handleNameSelectionMenu() {
        int dir = encoder_.readRotation();
        if (dir != 0) {
            if (dir > 0) {
                namePickerIndex_ = (namePickerIndex_ + 1) % NUM_FUTURAMA_NAMES;
            } else {
                namePickerIndex_ = (namePickerIndex_ == 0) ? NUM_FUTURAMA_NAMES - 1 : namePickerIndex_ - 1;
            }
            audio_.playMenuSelectSound();
            display_.renderNamePicker(namePickerIndex_);
        }

        if (encoder_.isButtonPressed()) {
            audio_.playMenuSelectSound();
            String chosen = FUTURAMA_NAMES[namePickerIndex_];
            mesh_.setNickname(chosen);
            
            // If auto-accepted as initial Host, immediately load lobby UI
            if (mesh_.getLocalStatus() == PlayerStatus::Accepted) {
                display_.renderLobby(mesh_.getLocalNickname(), mesh_.getPeerCount(), mesh_.getRegistry());
                display_.updateHUD(health_, ammo_);
            } else {
                joinScreenDrawn_ = false;
            }
        }
    }

    void handleJoinMenu() {
        if (!joinScreenDrawn_) {
            display_.renderJoinRequestScreen(mesh_.getLocalNickname());
            joinScreenDrawn_ = true;
        }

        if (encoder_.isButtonPressed()) {
            audio_.playMenuSelectSound();
            mesh_.broadcastIdentity();
            leds_.setMuzzleRingColor(0, 0, 255);
            delay(200);
            leds_.setMuzzleRingColor(0, 0, 0);
        }
    }

    void handleHostApprovalMenu(uint32_t applicantNodeId) {
        if (currentPromptApplicant_ != applicantNodeId) {
            currentPromptApplicant_ = applicantNodeId;
            acceptChoiceSelected_ = true;
            String applicantName = mesh_.getNicknameForNodeId(applicantNodeId);
            display_.renderAcceptPlayerPrompt(applicantName, acceptChoiceSelected_);
        }

        int dir = encoder_.readRotation();
        if (dir != 0) {
            acceptChoiceSelected_ = !acceptChoiceSelected_;
            audio_.playMenuSelectSound();
            String applicantName = mesh_.getNicknameForNodeId(applicantNodeId);
            display_.renderAcceptPlayerPrompt(applicantName, acceptChoiceSelected_);
        }

        if (encoder_.isButtonPressed()) {
            audio_.playMenuSelectSound();
            mesh_.sendJoinDecision(applicantNodeId, acceptChoiceSelected_);
            
            if (acceptChoiceSelected_) {
                leds_.setMuzzleRingColor(0, 255, 0);
            } else {
                leds_.setMuzzleRingColor(255, 0, 0);
            }
            delay(200);
            leds_.setMuzzleRingColor(0, 0, 0);

            currentPromptApplicant_ = 0;
            display_.renderLobby(mesh_.getLocalNickname(), mesh_.getPeerCount(), mesh_.getRegistry());
            display_.updateHUD(health_, ammo_);
        }
    }

    void handleInGameLoop() {
        handleTrigger();
        handleIRReception();
        handleReloadGesture();
    }

    void handleBatteryAndSignalMonitoring() {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastBatteryCheck_).count() >= 2000) {
            lastBatteryCheck_ = now;
            display_.updateBattery(battery_.getPercentage());
            
            // RSSI shifted +2px right to X=154
            display_.drawSignalIcon(154, 16, mesh_.getRssi());
        }
    }

    void handleTrigger() {
        if (health_ == 0) return;

        bool currentTriggerState = digitalRead(Pins::Trigger);
        auto now = std::chrono::steady_clock::now();
        
        if (lastTriggerState_ == HIGH && currentTriggerState == LOW && 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - lastShotTime_).count() >= FireRateLimitMs) {
            
            if (ammo_ > 0) {
                ammo_--;
                lastShotTime_ = now;
                ir_.fireShot();
                leds_.triggerMuzzleFlash(40);
                audio_.playFireSound();
                display_.updateHUD(health_, ammo_);
            } else {
                leds_.setMuzzleRingColor(255, 140, 0);
                delay(30);
                leds_.setMuzzleRingColor(0, 0, 0);
            }
        }
        lastTriggerState_ = currentTriggerState;
    }

    void handleReloadGesture() {
        if (health_ == 0) return;

        if (motion_.detectTiltReload() && ammo_ < 30) {
            ammo_ = 30;
            leds_.setMuzzleRingColor(0, 255, 0);
            delay(100);
            leds_.setMuzzleRingColor(0, 0, 0);
            audio_.playReloadSound();
            display_.updateHUD(health_, ammo_);
        }
    }

    void handleIRReception() {
        if (health_ == 0) return;

        uint16_t attackerIrId {0};
        if (ir_.checkForHit(attackerIrId)) {
            if (health_ > 0) health_--;
            display_.updateHUD(health_, ammo_);
            leds_.triggerHitEffect(health_, 350);
            audio_.playHitSound();
        }
    }
};

// ============================================================================
// 12. ENTRY POINT
// ============================================================================
LaserTagApp app;

void setup() {
    app.setup();
}

void loop() {
    app.loop();
}