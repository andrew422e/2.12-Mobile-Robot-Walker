import time
import pygame
import serial

pygame.init()
pygame.joystick.init()

ser = serial.Serial('COM9', 115200, timeout=0.05)
time.sleep(2)
print("Serial connection established")

joystick = pygame.joystick.Joystick(0)
joystick.init()

class BluetoothJoystick:
    def __init__(self, joystick):
        self.joystick = joystick
        self.buttonF = 0.0
        self.buttonB = 0.0
        self.buttonL = 0.0
        self.buttonR = 0.0
        self.axisX = 0.0
        self.axisY = 0.0

        self.prevbuttonF = 0.0
        self.prevbuttonB = 0.0
        self.prevbuttonL = 0.0
        self.prevbuttonR = 0.0
        self.prevaxisX = 0.0
        self.prevaxisY = 0.0

        self.filteredAxisX = 0.0
        self.filteredAxisY = 0.0
        self.alpha = 0.5

    def update_previous_state(self):
        self.prevbuttonF = self.buttonF
        self.prevbuttonB = self.buttonB
        self.prevbuttonL = self.buttonL
        self.prevbuttonR = self.buttonR
        self.prevaxisX = self.axisX
        self.prevaxisY = self.axisY

    def has_state_changed(self):
        return (self.buttonF != self.prevbuttonF or
                self.buttonB != self.prevbuttonB or
                self.buttonL != self.prevbuttonL or
                self.buttonR != self.prevbuttonR or
                abs(self.axisX - self.prevaxisX) > 0.1 or
                abs(self.axisY - self.prevaxisY) > 0.1)

    def print_state(self):
        # Here you would implement the Bluetooth communication to send the state
        # For demonstration, we'll just print the state
        print(f"Buttons: F={self.buttonF}, B={self.buttonB}, L={self.buttonL}, R={self.buttonR}\n")
        print(f"Axes: X={self.axisX}, Y={self.axisY}\n")

    def print_to_serial(self):
        payload = (
            f"{self.filteredAxisX:.3f},"
            f"{self.filteredAxisY:.3f},"
            f"{self.buttonF:.1f},"
            f"{self.buttonB:.1f},"
            f"{self.buttonL:.1f},"
            f"{self.buttonR:.1f}\n"
        )
        ser.write(payload.encode("ascii"))
        ser.flush()

    def update_filtered_axes(self):
        self.filteredAxisX = self.alpha * self.axisX + (1 - self.alpha) * self.filteredAxisX
        self.filteredAxisY = self.alpha * self.axisY + (1 - self.alpha) * self.filteredAxisY



bluetooth_joystick = BluetoothJoystick(joystick)
SEND_PERIOD_S = 0.05
last_send_time = 0.0

while True:
    pygame.event.pump()  # update state

    # Buttons
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i):
            if i == 0:  # Button A
                bluetooth_joystick.buttonR = 1.0
            elif i == 1:  # Button B
                bluetooth_joystick.buttonF= 1.0
            elif i == 2:  # Button X
                bluetooth_joystick.buttonB = 1.0
            elif i == 3:  # Button Y
                bluetooth_joystick.buttonL = 1.0
        else:
            if i == 0:
                bluetooth_joystick.buttonR = 0.0
            elif i == 1:
                bluetooth_joystick.buttonF = 0.0
            elif i == 2:
                bluetooth_joystick.buttonB = 0.0
            elif i == 3:
                bluetooth_joystick.buttonL = 0.0

    # Axes (joysticks)
    for i in range(joystick.get_numaxes()):
        val = joystick.get_axis(i)
        if abs(val) > 0.1:  # deadzone
            if i == 0:  # Left stick X
                bluetooth_joystick.axisX = val
            elif i == 1:  # Left stick Y
                bluetooth_joystick.axisY = -val
        else:
            if i == 0:
                bluetooth_joystick.axisX = 0.0
            elif i == 1:
                bluetooth_joystick.axisY = 0.0

    bluetooth_joystick.update_filtered_axes()

    now = time.monotonic()
    if now - last_send_time >= SEND_PERIOD_S:
        bluetooth_joystick.print_state()
        bluetooth_joystick.print_to_serial()
        last_send_time = now

    while ser.in_waiting > 0:
        msg = ser.read_until(b'\n').decode(errors='ignore').strip()
        if msg:
            print(msg)

    bluetooth_joystick.update_previous_state()
    time.sleep(0.005)
