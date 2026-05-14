// MeshEvents - transient events across a mesh group.
//
// Unlike properties (which are replicated and resent until acknowledged
// by snapshot), events are fire-and-forget broadcasts to the current
// group. This sketch sets up two channels:
//
//   - kButtonChannel : EventHandle<void>      - cheap signal, no payload
//   - kAlertChannel  : EventHandle<AlertMsg>  - typed struct payload
//
// Flash the same firmware onto two or more boards. The board that ends
// up as group master (AUTO mode picks one) emits an "alert" every few
// seconds; every node emits a "button" event every second; every node
// logs both. Press the BOOT button on an ESP32 if you want to trigger a
// real button event - GPIO0 reads LOW when pressed.

#include <Arduino.h>
#include "Mesh.h"
#include "Wireless.h"

constexpr uint16_t kButtonChannel = 0x7301;
constexpr uint16_t kAlertChannel  = 0x7302;
constexpr uint8_t  kButtonPin     = 0; // BOOT button on most ESP32 dev boards

enum class AlertLevel : uint8_t
{
  Info = 0,
  Warn = 1,
  Crit = 2,
};

struct AlertMsg
{
  uint32_t   id;
  AlertLevel level;
  uint8_t    code;
  uint16_t   reserved;
};

SyncManager *mesh = nullptr;
EventHandle<void>     buttonEvent;
EventHandle<AlertMsg> alertEvent;

uint32_t lastTickMs   = 0;
uint32_t lastAlertMs  = 0;
uint32_t alertCounter = 0;
bool     lastBtn      = HIGH;

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("== MeshEvents ==");

  pinMode(kButtonPin, INPUT_PULLUP);

  mesh = SyncManager::getInstance();
  mesh->setTransport(Wireless::getInstance());
  mesh->setSyncMode(SyncMode::AUTO);
  mesh->begin();

  buttonEvent = mesh->event<void>(kButtonChannel);
  alertEvent  = mesh->event<AlertMsg>(kAlertChannel);

  buttonEvent.onEvent([](uint32_t fromDevice) {
    Serial.printf("[button] from 0x%08lx\n",
                  static_cast<unsigned long>(fromDevice));
  });

  alertEvent.onEvent([](uint32_t fromDevice, AlertMsg m) {
    const char *label =
        m.level == AlertLevel::Crit ? "CRIT" :
        m.level == AlertLevel::Warn ? "WARN" : "INFO";
    Serial.printf("[alert] from 0x%08lx id=%lu level=%s code=%u\n",
                  static_cast<unsigned long>(fromDevice),
                  static_cast<unsigned long>(m.id),
                  label,
                  static_cast<unsigned>(m.code));
  });
}

void loop()
{
  mesh->loop();
  if (!mesh->isInGroup()) return;

  const uint32_t now = millis();

  // BOOT button edge -> emit a void event.
  const bool btn = digitalRead(kButtonPin);
  if (lastBtn == HIGH && btn == LOW)
  {
    Serial.println("local button pressed -> emit");
    buttonEvent.emit();
  }
  lastBtn = btn;

  // Heartbeat-style emit so the example does something even without
  // touching the board.
  if (now - lastTickMs >= 1000)
  {
    lastTickMs = now;
    buttonEvent.emit();
  }

  // Only the master raises alerts.
  if (mesh->isGroupMaster() && now - lastAlertMs >= 3000)
  {
    lastAlertMs = now;
    AlertMsg m{
        ++alertCounter,
        static_cast<AlertLevel>(alertCounter % 3),
        static_cast<uint8_t>(alertCounter & 0xFF),
        0};
    alertEvent.emit(m);
  }
}
