#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "robot_pinout.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"
#include "imu.h"

// #define UTURN
// #define CIRCLE
#define JOYSTICK
// #define HOMING
// #define YOUR_TRAJECTORY

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

IMU imu(BNO08X_RESET, BNO08X_CS, BNO08X_INT);
double heading = 0.0;

int state = 0;
double robotVelocity = 0; // velocity of robot, in m/s
double k = 0; // curvature k is 1/radius from center of rotation circle

extern EncoderVelocity encoders[NUM_MOTORS];
double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
void setWheelVelocities(float robotVelocity, float k){
    double left = (robotVelocity - k * BACK_WHEEL_TO_CENTER * robotVelocity) / WHEEL_RADIUS;
    double right = 2 * robotVelocity / WHEEL_RADIUS  - left;
    updateSetpoints(left, right);
}


// Makes robot follow a trajectory
void followTrajectory() {

    #ifdef JOYSTICK
    bool hasFreshWirelessData = freshWirelessData;

    if (hasFreshWirelessData) {
        freshWirelessData = false;
    }


    if (hasFreshWirelessData) {
        Serial.printf("Joystick x: %.2f, y: %.2f\n",
                      controllerMessage.joystick1.x,
                      controllerMessage.joystick1.y);
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
        double turn = 2 * (abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN));
        updateSetpoints(forward + turn, forward - turn);
    }

    //     switch (state) {
    //         case 0:{
    //             if (hasFreshWirelessData) {
    //                 Serial.println("Received new controller data!");
    //                 double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
    //                 double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);
    //                 updateSetpoints(forward + turn, forward - turn);
    //                 directSetpointsUpdated = true;
    //             }

    //             if (controllerMessage.buttonL) {
    //                 Serial.println("Button L pressed!");
    //                 state = 1;
    //             }
    //             break;
    //             }
    //         case 1:
    //             zeroHeading = atan2(robotMessage.y, robotMessage.x);

    //             if (abs(robotMessage.theta - zeroHeading)>0.1) {
    //                 // Turn in a circle with radius 0 cm
    //                 robotVelocity = 0.2;
    //                 k = 1 / 0.01;
    //             } else {
    //                 state = 2;
    //             }
    //             break;


    //         case 2:
    //             if (abs(robotMessage.x) > 0.1 || abs(robotMessage.y) > 0.1) {
    //                 // Move in a straight line forward
    //                 robotVelocity = 0.2;
    //                 k = 0;
    //             } else {
    //                 state = 0;
    //             }
    //             break;


    //         default:
    //             // If not in any of the states, robot should just stop
    //             robotVelocity = 0;
    //             k = 0;
    //             break;
    //         }

    // if (!directSetpointsUpdated) {
    //     setWheelVelocities(robotVelocity, k);
    // }
    #endif

    #ifdef CIRCLE
    robotVelocity = 0.2;
    k = 1/0.5;
    setWheelVelocities(robotVelocity, k);
    #endif



    #ifdef YOUR_TRAJECTORY
    // TODO: Create a state machine to define your custom trajectory!
    switch (state) {
        case 0:
            // Until robot has achieved an x translation of 1 m:
            if (robotMessage.x <= 1.0) {
                // Move in a straight line forward
                robotVelocity = 0.5;
                k = 0;
            } else {
                // Move on to next state
                Serial.print("switching to state 1");
                state++;

            }
            break;

        case 1:
            // Until robot has achieved a 180 deg turn in theta:
            heading = imu.getEulerAngles().yaw;
            Serial.print(imu.getEulerAngles().yaw);
            if (heading <= M_PI) {
                // Turn in a circle with very small radius
                robotVelocity = 0.01;
                k = 1 / 0.01;
            } else {
                Serial.print("switching to state 2\n");
                state++;
            }
            break;

        case 2:
            // Until robot has achieved an x translation of -1 m:
            if (robotMessage.x >= 0) {
                // Move in a straight line forward
                robotVelocity = 0.5;
                k = 0;
            } else {
                // Move on to next state
                Serial.print("switching to state 3");
                state++;
            }
            break;

        case 3:
            // until robot has acheived a rotation of -180 degrees in theta
            heading = imu.getEulerAngles().yaw;
            if (heading >= 0){
                // turn in circle with very small radius
                robotVelocity = 0.01;
                k = 1/0.01;
            } else {
                robotMessage.theta = 0;
                Serial.print("switching to state 0");
                state = 0;
            }
            break;

        default:
            // If not in any of the states, robot should just stop
            robotVelocity = 0;
            k = 0;
            break;
    }
    setWheelVelocities(robotVelocity, k);
    #endif

}

void updateOdometry() {
    // Take angles from traction (rear) wheels only since they don't slip
    currPhiL = encoders[2].getPosition();
    currPhiR = -encoders[3].getPosition();

    // Update wheel angles and angular change
    double dPhiL = currPhiL - prevPhiL;
    double dPhiR = currPhiR - prevPhiR;
    prevPhiL = currPhiL;
    prevPhiR = currPhiR;

    // Calculate update in robot's base coordinates
    float dtheta = WHEEL_RADIUS / (2 * BACK_WHEEL_TO_CENTER) * (dPhiR - dPhiL);
    float dx = WHEEL_RADIUS / 2.0 * (cos(robotMessage.theta) * dPhiR + cos(robotMessage.theta) * dPhiL);
    float dy = WHEEL_RADIUS / 2.0 * (sin(robotMessage.theta) * dPhiR + sin(robotMessage.theta) * dPhiL);

    // Update robot message
    robotMessage.millis = millis();
    robotMessage.x += dx;
    robotMessage.y += dy;
    robotMessage.theta += dtheta;
}
