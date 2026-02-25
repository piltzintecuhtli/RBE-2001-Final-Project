/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

void Robot::UpdatePose(const Twist& twist)
{
    float time = 0.020; // 20 ms update time

    float newTheta = currPose.theta + twist.omega * time;
    float avgTheta = (currPose.theta + newTheta) / 2;

    float newx = currPose.x + time * twist.u * cos(avgTheta);
    float newy = currPose.y + time * twist.u * sin(avgTheta);

    currPose.x = newx;
    currPose.y = newy;
    currPose.theta = newTheta;

#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
    TeleplotPrintXY("pose", currPose.x, currPose.y);
#endif

}

float clampReal(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float invClamp(float value, float min, float max) {
    if (value > min && value < max) {
        return (value - min > max - value) ? max : min;
    }
    return value;
}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

float drivekP = 10;
float turnkP = 10;

void Robot::Spin(void) {
    if(robotState == ROBOT_DRIVE_TO_POINT) {
        float error = destPose.theta - currPose.theta;
        float effortLeft = invClamp(-error * turnkP, -25, 25);
        float effortRight = invClamp(error * turnkP, -25, 25);
        chassis.SetMotorEfforts(effortLeft, effortRight);
    }
}

bool Robot::CheckSpin(void) {
    return fabs(destPose.theta - currPose.theta) < 0.05; // error tolerance of 0.2 radians
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        float errHead = fmod(atan2(destPose.y - currPose.y, destPose.x - currPose.x) - currPose.theta, 2*PI);
        errHead -= (errHead > PI) ? 2 * PI : 0;
        
        float errDist = sqrt(pow(destPose.x - currPose.x, 2) + pow(destPose.y - currPose.y, 2)) * cos(errHead);

        TeleplotPrint("errDist", errDist);

        float effortLeft = clampReal(invClamp(clampReal(errDist * drivekP, -60, 60), -20, 20) - invClamp(errHead * turnkP, -20, 20), -60, 60);
        float effortRight = clampReal(invClamp(clampReal(errDist * drivekP, -60, 60), -20, 20) + invClamp(errHead * turnkP, -20, 20), -60, 60);

         TeleplotPrint("effortLeft", effortLeft);
         TeleplotPrint("error head", errHead);

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

        chassis.SetMotorEfforts(effortLeft, effortRight);
    }
}

bool Robot::CheckReachedDestination(void)
{
    return sqrt(pow(destPose.x - currPose.x, 2) + pow(destPose.y - currPose.y, 2)) < 2.0; // error tolerance of 2 cm
}

void Robot::HandleDestination(void)
{
    chassis.Stop();
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(13, LOW);
}