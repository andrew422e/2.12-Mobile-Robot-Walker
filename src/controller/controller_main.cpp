#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"

ControllerMessage prevControllerMessage;
unsigned long lastSerialControllerUpdate = 0;

constexpr unsigned long SERIAL_CONTROL_TIMEOUT_MS = 200;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);
Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN);

void setup() {
    Serial.begin(115200);

    setupWireless();


    joystick1.setup();

    // Serial.println("Setup complete.");
}

void loop() {
    // Read and send controller sensors
    EVERY_N_MILLIS(50) {
        bool receivedSerialControllerData = false;

        if (Serial.available()) {
            String msg = Serial.readStringUntil('\n');
            msg.trim();
            if (controllerMessage.readFromSerialLine(msg)) {
                controllerMessage.updateControllerNewData();
                lastSerialControllerUpdate = millis();
                receivedSerialControllerData = true;
            }
        }

        controllerMessage.millis = millis();

        // Prefer recent Bluetooth/serial joystick input. Fall back to the wired joystick
        // when no fresh serial packet has arrived recently.
        if (!receivedSerialControllerData &&
            millis() - lastSerialControllerUpdate > SERIAL_CONTROL_TIMEOUT_MS) {
            controllerMessage.joystick1 = joystick1.read();
            controllerMessage.buttonL = digitalRead(BUTTON_L_PIN);
        }

        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}
