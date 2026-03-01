#include "robot.h"
#include <Romi32U4Buttons.h>
#include <servo32u4.h>
#include <stdlib.h>

Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

Servo32U4Pin5 baseServo;
Servo32U4Pin12 clawServo;
Servo32U4Pin13 armServo;

int taskTimer = 0;

float angleToMs(float angle) {
    // -90 is 1 ms, 0 is 1.5 ms, 90 is 2 ms
    return (angle / 180.0) + 1.5;     
}

float armPoses[] = {
    30.0, // floor pickup
    45.0, // shelf 1
    50.0, // shelf 2
    55.0, // shelf 3
    -15.0 // final
};

float retractPose = -45.0;

float baseArmPoses[] = {
    10.0, 
    retractPose, 
    -20.0, 
    retractPose,
    -15.0, 
    retractPose,
    0.0, 
    -70.0
};

float clawPoses[] = {
    angleToMs(20.0), // close
    angleToMs(0.0) // open
};

void moveServo(Servo32U4Base& servo, float angle) {
    servo.setTargetPos(angleToMs(angle));
    servo.update();
    delay(1000);
}

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();
    baseServo.attach();
    clawServo.attach();
    armServo.attach();

    /**
     * TODO: Set pin 13 HIGH when navigating and LOW when destination is reached.
     * Need to set as OUTPUT here.
     */
    pinMode(13, OUTPUT);
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}


float poses[][2] = {{0, 0}, {20, 0}, {20, -20}, {0, -20}};
int poseIndex = 0;

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (distance readings, etc.).
*/

Pose pose;
bool spinned = false;

void Robot::RobotLoop(void) 
{
        /**
         * Run the chassis loop, which handles low-level control.
         */
        Twist velocity;
        if(chassis.ChassisLoop(velocity)) {
            // We do FK regardless of state
            UpdatePose(velocity);
                
            /**
             * Here, we break with tradition and only call these functions if we're in the 
             * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
             * to do all the maths when we don't need to.
             * 
             * While we're at it, we'll toss DriveToPoint() in, as well.
             */ 
            if (robotState == ROBOT_TASK_WAIT) {
                taskTimer--;
                if (taskTimer <= 0) {
                    SetDestination(pose);
                }
            }
            else if(buttonC.isPressed()) {
                taskTimer = 30;
                robotState = ROBOT_TASK;
                pose = Pose(poses[poseIndex][0], poses[poseIndex][1], 0);
            }
            if(robotState == ROBOT_TASK) {
                // move arm to first block and hold it
                moveServo(armServo, armPoses[0]);
                moveServo(baseServo, baseArmPoses[0]);
                moveServo(clawServo, clawPoses[0]);

                moveServo(baseServo, baseArmPoses[1]);
                moveServo(armServo, armPoses[1]);

                moveServo(baseServo, baseArmPoses[2]);
                moveServo(clawServo, clawPoses[1]); // release block on first shelf

                moveServo(baseServo, baseArmPoses[3]);
                moveServo(armServo, armPoses[2]);

                moveServo(baseServo, baseArmPoses[4]);
                moveServo(clawServo, clawPoses[0]); // grab block on second shelf

                moveServo(baseServo, baseArmPoses[5]);
                moveServo(armServo, armPoses[3]);

                moveServo(baseServo, baseArmPoses[6]);
                moveServo(clawServo, clawPoses[1]); // release block on final shelf

                moveServo(baseServo, baseArmPoses[7]);
                moveServo(armServo, armPoses[4]); // move arm out of the way

                robotState = ROBOT_TASK_WAIT;
            }
            if(robotState == ROBOT_DRIVE_TO_POINT) {

                // spin to face the point to minimize turning while driving
                
                float xErr = destPose.x - currPose.x;
                float yErr = destPose.y - currPose.y;
                float errHead = atan2(yErr, xErr) - currPose.theta;
                errHead = fmod(errHead, 2 * PI);
                errHead -= (errHead > PI) ? 2 * PI : 0;

                Pose tempPose = Pose(0, 0, errHead);

                if(!spinned) {
                    SetDestination(tempPose);
                    Spin();
                    if(CheckSpin()) {
                        HandleDestination();
                        spinned = true;
                    }
                }
                // drive to point with hopefully minimal turning
                else {
                    SetDestination(pose);
                    DriveToPoint();
                    if(CheckReachedDestination()) {
                        HandleDestination();
                        // if at destination, update to the next pose in the list
                        if(poseIndex <= sizeof(poses)/sizeof(poses[0])) {
                            poseIndex++;
                            pose = Pose(poses[poseIndex][0], poses[poseIndex][1], 0);
                        }
                    }
                }
            }
        }
    }

