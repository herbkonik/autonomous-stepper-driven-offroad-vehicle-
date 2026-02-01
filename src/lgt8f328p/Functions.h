#pragma once

void setupTimer();
void setupMotors();
void setupServos();
void setupI2C();
void setMotorSpeed(byte motorID, int stepsPerSec);
void updateMotorSpeed(byte motorID);
void stopMotor(byte motorID);
void emergencyStopMotor(byte motorID);
void setServoAngle(byte servoID, int angle);
void setServoAngle(byte servoID, float angle);
void updateServoAngle(byte servoID);
void setAllMotorsSpeed(int speed);
void setLeftSideSpeed(int speed);
void setRightSideSpeed(int speed);
void setAllSteering(int angle);
void setAllSteering(float angle);
void stopAllMotors();
void emergencyStopAll();
void centerAllSteering();
void driveForward(int speed);
void driveBackward(int speed);
void turnLeft(int baseSpeed, int speedDifference);
void turnRight(int baseSpeed, int speedDifference);
void steerLeft(int angle);
void steerLeft(float angle);
void steerRight(int angle);
void steerRight(float angle);
void stop();
void i2cReceiveEvent(int numBytes);   
void processRegisters();  

