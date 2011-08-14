/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
# Ziptie
# Copyright (C) 2011 John Moore and Jeremy Lopez
#
# http://programiscellaneous.com/programming-projects/first-robotics-ziptie/what-is-it/
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/ 
*/

package edu.wpi.first.wpilibj.defaultCode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.AnalogModule;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Gyro;


/* CONTROLS
 *
 * Arm Joystick:
 *      T: Camera/claw control toggle for buttons 2-5
 *      J: Arm up/down
 *      2: If T, open claw, else camera down
 *      3: If T, close claw, else camera up
 *      4: Camera left (regardless of T)
 *      5: Camera right (regardless of T)
 *      Z: Arm power
 *      6: Arm -- go to peg 2 of shorter (side) post
 *      7: Arm -- go to peg 1 of shorter (side) post
 *      8: Camera reset
 *      9: Arm -- go to carry position
 *      10: Arm -- go to peg 1 of taller (center) post
 *      11: Arm -- go to peg 2 of taller (center) post
 *
 * Drive Joystick:
 *      J: Arcade mode robot steering
 *      Z: Robot power
 *      3: Increase turning sensitivity (Overrides DS temporarily)
 *      2: Decrease turning sensitivity (Overrides DS temporarily)
 *      4: Set turning sensitivity to scoring preset
 *      5: Set turning sensitivity to turning preset
 *
 * Digital Out:
 *      1. Line Left
 *      2. Line Center
 *      3. Line Right
 *
 * Digital In:
 *      1. Straight Line of Tape at Start
 *      2. Take Right at Fork
 *      3. Use cubic speed regression
 *      4. Go for bottom peg during autonomous
 *      8. Attempt autonomous
 *
 * Other:
 *      Encoder -- Right drive motor
 *      Encoder -- Arm
 *
 * DIGITAL SIDECAR
 *      DI1: Left Line Sensor
 *      DI2: Center Line Sensor
 *      DI3: Right Line Sensor
 *      DI4: Right Drive Encoder A
 *      DI5: Right Drive Encoder B
 *      DI6: Arm Encoder A
 *      DI7: Arm Encoder B
 *
 *      PWM1: Left Drive Motor
 *      PWM2: Right Drive Motor
 *      PWM7: Camea Servo Tilt
 *      PWM8: Camera Servo Pan
 *      PWM5: Arm window motors (Y-splitter)
 *      PWM6: Claw windo motor
 *
 */


public class DefaultRobot extends IterativeRobot {


    //Control Constants -- Arm
    static final int BUTTON_CAMERA_DOWN = 2;
    static final int BUTTON_CAMERA_UP = 3;
    static final int BUTTON_CAMERA_LEFT = 4;
    static final int BUTTON_CAMERA_RIGHT = 5;
    static final int BUTTON_ARM_SIDE_PEG1 = 7;
    static final int BUTTON_ARM_SIDE_PEG2 = 6;
    static final int BUTTON_ARM_CENTER_PEG1 = 10;
    static final int BUTTON_ARM_CENTER_PEG2 = 11;
    static final int BUTTON_ARM_CARRY = 9;
    static final int BUTTON_CAMERA_RESET = 8;
    static final int BUTTON_CLAW_TRIGGER_OPEN = 2;
    static final int BUTTON_CLAW_TRIGGER_CLOSE = 3;

    //Control Constants -- Drive
    final static int BUTTON_ROBOT_SENS_INC = 3;
    final static int BUTTON_ROBOT_SENS_DEC = 2;
    final static int BUTTON_ROBOT_SENS_SCORE = 4;
    final static int BUTTON_ROBOT_SENS_TURN = 5;

    //Digital Sidecar Constants
    static final int SENSOR_LINE_LEFT = 1;
    static final int SENSOR_LINE_CENTER = 2;
    static final int SENSOR_LINE_RIGHT = 3;

    static final int ENCODER_DRIVE_A = 4;
    static final int ENCODER_DRIVE_B = 5;
    static final int ENCODER_ARM_A = 6;
    static final int ENCODER_ARM_B = 7;

    static final int MOTOR_DRIVE_LEFT = 1;
    static final int MOTOR_DRIVE_RIGHT = 2;
    static final int MOTOR_ARM = 5; //Two Jaguars on a Y-splitter PWM cable
    static final int SERVO_CAMERA_TILT = 7;
    static final int SERVO_CAMERA_PAN = 8;
    static final int MOTOR_CLAW = 9;

    static final int JOYSTICK_RIGHT = 1;
    static final int JOYSTICK_ARM = 2;

    //IO Constants
    static final int DI_PATH = 1; //fork vs. straight for tape path at start
    static final int DI_FORKDIR = 2; //right or left at fork
    static final int DI_SPEEDREG = 3; //use cubic speed reg
    static final int DI_AUTOGOFORBOTTOM = 4; //go for bottom peg in autonomous? if false, go for middle peg
    static final int DI_AUTOATTEMPT = 8; //attempt autonomous mode?

    static final int AI_SENSITIVITY = 1;
    static final int AI_STEERGAIN = 2;
    static final int AI_ARMSPEED = 3;
    static final int AI_DOWNFACTOR = 4;

    static final int DO_LEFTLINE = 1;
    static final int DO_CENTERLINE = 2;
    static final int DO_RIGHTLINE = 3;

    //Other Constants
    static final double SPEED_CLAW_OPEN = 0.7;
    static final double SPEED_CLAW_CLOSE = -0.5;
    static final double SPEED_CLAW_MAX_OPEN = 1;
    static final double SPEED_CLAW_MAX_CLOSE = -1;

    final static double RANGE_POWER_DRIVE_LOW = .15;
    final static double RANGE_POWER_DRIVE_HIGH = 1;
    final static double RANGE_POWER_ARM_LOW = 0.5;
    final static double RANGE_POWER_ARM_HIGH = 1;

    final static double DEFAULT_CAMERA_TILT = 90; //degrees
    final static double DEFAULT_CAMERA_PAN = 90; //degrees
    final static double CAMERA_INCREMENT_PAN = 5;
    final static double CAMERA_INCREMENT_TILT = 5;

    final static double SENS_PRESET_SCORE = 0.59;
    final static double SENS_PRESET_TURN = 0.69;

    //Autonomous Constants -- Important!
    final static double AUTONOMOUS_ARM_DROP_DIST = 20; //distance to drop arm at end of autonomous
    final static double AUTONOMOUS_TIME_STOP_STRAIGHT = 7; //when to expect the fork
    final static double AUTONOMOUS_TIME_STOP_FORK = 0; //time to stop if taking fork
    final static double AUTONOMOUS_TIME_FORK = 5;
    final static double[] AUTONOMOUS_PROFILE_TIME_FORK = {0.8, 0.65, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4}; //in half-second intervals
    final static double[] AUTONOMOUS_PROFILE_TIME_STRAIGHT = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4}; //in half-second intervals
    final static double[][] AUTONOMOUS_PROFILE_DIST_FORK = {{1000, 0.5}, {1500, 0.4}, {2000, 0.3}, {2500, 0}};
    final static double[][] AUTONOMOUS_PROFILE_DIST_STRAIGHT = {{0, 0.7}, {1000, 0.7}, {2500, 0.5}, {4000, 0.3}, {5200, 0}};
    final static double AUTONOMOUS_SPEED_FORK = 0.45; //speed to travel at fork
    final static double AUTONOMOUS_DIST_FORK = 1000; //Right drive encoder distance until fork
    final static boolean AUTONOMOUS_USE_DIST_METHOD = false; //use distance profiles instead?
    final static double AUTONOMOUS_Kp = 0.03; //turning constant for gyro

    //Counters & Misc
    int driveMode;
    boolean clawOpen = true; //starts open
    boolean clawTriggerDown = false;
    boolean startingAtFork = false;
    boolean goRightAtFork = false;
    boolean useSpeedReg = false;
    boolean sensitivityOverride;

    double armPower = 1;
    double robotPower = 1;
    double downFactor = 1;
    double overriddenSensitivity;
    double sensitivityIncrement = 0.025;
    double robotSensitivity = 1;
    double oldSensitivity = 1;
    static int printSec;
    static int startSec;

    //Main Objects
    RobotDrive robotDrive;
    AutonomousEnabledArm arm;
    Victor claw;

    Joystick rightStick;
    Joystick leftStick;
    Joystick armStick;

    AxisCamera camera;

    DriverStation ds;

    Servo cameraPan;
    Servo cameraTilt;

    DigitalInput leftLineSensor;
    DigitalInput centerLineSensor;
    DigitalInput rightLineSensor;

    Encoder rightDriveEncoder;

    Gyro robotGyro;


    public DefaultRobot() {
        System.out.println("Ziptie Initializing...\n");

        robotDrive = new RobotDrive(MOTOR_DRIVE_LEFT, MOTOR_DRIVE_RIGHT);

        rightStick = new Joystick(JOYSTICK_RIGHT);
        armStick = new Joystick(JOYSTICK_ARM);


        claw = new Victor(MOTOR_CLAW);
        arm = new AutonomousEnabledArm(MOTOR_ARM, ENCODER_ARM_A, ENCODER_ARM_B);
        arm.setInvertedMotor(RobotArm.MotorType.kArm, true);

        ds = DriverStation.getInstance();

        cameraPan = new Servo(SERVO_CAMERA_PAN);
        cameraTilt = new Servo(SERVO_CAMERA_TILT);

        leftLineSensor = new DigitalInput(SENSOR_LINE_LEFT);
        centerLineSensor = new DigitalInput(SENSOR_LINE_CENTER);
        rightLineSensor = new DigitalInput (SENSOR_LINE_RIGHT);

        rightDriveEncoder = new Encoder(new DigitalInput(ENCODER_DRIVE_A), new DigitalInput(ENCODER_DRIVE_B), false, CounterBase.EncodingType.k4X);

        robotGyro = new Gyro(1, 1);
        robotGyro.setSensitivity(0.01);
        int buttonNum = 1;

        System.out.println("Ziptie Online!\n");
    }

    

    public void robotInit() {
            // Actions which would be performed once (and only once) upon initialization of the
            // robot would be put here.
            System.out.println("RobotInit() completed.\n");
            robotGyro.reset();
    }

    public void disabledInit() {
        rightDriveEncoder.stop();
        rightDriveEncoder.reset();
        arm.resetArmEncoder();
        robotGyro.reset();
    }

    public void autonomousInit() {
        rightDriveEncoder.start();
        System.out.println("Autonomous mode started");
        if (ds.getDigitalIn(DI_AUTOATTEMPT)) {
            //close claw
            Timer timer = new Timer();
            timer.start();
            while (timer.get() < 0.5) {
                claw.set(SPEED_CLAW_MAX_CLOSE);
            }
            claw.set(0);
            //drop arm
            //arm.goToCarryPosition(scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1));
            //now bring arm up
            arm.goToPeg(ds.getDigitalIn(DI_AUTOGOFORBOTTOM), (ds.getDigitalIn(DI_PATH) ? true : false), scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1));
            //drive to the grid, and be careful...
            //goToScoreGrid(); //ugh
            double angle = robotGyro.getAngle();
            arm.arcadeDrive(0.31, 0, 0.25);
            while (rightDriveEncoder.getDistance() < 4000) {
                angle = robotGyro.getAngle();
                Watchdog.getInstance().feed();
                robotDrive.arcadeDrive(-0.7, -angle * 0.00);
                Timer.delay(0.004);
            }
            robotDrive.drive(0, 0);
            //re-raise the arm in case it fell
            //arm.goToPeg(ds.getDigitalIn(DI_AUTOGOFORBOTTOM), (ds.getDigitalIn(DI_PATH) ? true : false), scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1));
            Timer.delay(2.0);
            timer.reset();
            timer.start();
            while (timer.get() < 1) {
                robotDrive.arcadeDrive(-0.5, 0);
                arm.arcadeDrive(0.31, 0, 0.25);
            }
            robotDrive.drive(0, 0);
            arm.arcadeDrive(0, 0, 0.25);
            arm.lower(AUTONOMOUS_ARM_DROP_DIST, scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1));

//lower arm a specified distance and make our exit
            //arm.lower(AUTONOMOUS_ARM_DROP_DIST, scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1));
//arm.arcadeDrive(0.31, 0, 0.25);
            //open claw
            timer.reset();
            timer.start();
            while (timer.get() < 1) {
                claw.set(SPEED_CLAW_MAX_OPEN);
            }
            claw.set(0);
            //lower arm a specified distance and make our exit
            arm.lower(AUTONOMOUS_ARM_DROP_DIST, scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1));
        }
    }

    public void teleopInit() {
            arm.resetArmEncoder();
            arm.startArmEncoder();
            rightDriveEncoder.start();
            cameraTilt.setAngle(DEFAULT_CAMERA_TILT);
            cameraPan.setAngle(DEFAULT_CAMERA_PAN);
    }

    public void disabledPeriodic()  {
            // feed the user watchdog at every period when disabled
            Watchdog.getInstance().feed();

            // while disabled, printout the duration of current disabled mode in seconds
            if ((Timer.getUsClock() / 1000000.0) > printSec) {
                    System.out.println("Disabled seconds: " + (printSec - startSec));
                    printSec++;
            }
            feedOutput();
    }

    public void autonomousPeriodic() {
            // feed the user watchdog at every period when in autonomous
            Watchdog.getInstance().feed();
    }

    public void teleopPeriodic() {
        // feed the user watchdog at every period
        Watchdog.getInstance().feed();
        updateDashboard();

        handleRobotMovement();
        handleArmMovement();
        handleCameraMovement();
        handleClawMovement();
        feedOutput();
        processInput();
        
    }

    private void handleRobotMovement() {
        robotDrive.arcadeDrive(cubicSpeedReg(rightStick.getY()), cubicSpeedReg(rightStick.getX() * robotSensitivity), true); //scales the X-axis component
    }

    private void handleArmMovement() {
        if (armStick.getRawButton(BUTTON_ARM_CENTER_PEG1)) {
            arm.goToPeg(true, true, scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), downFactor);
            Timer.delay(1);
        } else if (armStick.getRawButton(BUTTON_ARM_CENTER_PEG2)) {
            arm.goToPeg(false, true, scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), downFactor);
            Timer.delay(1);
        } else if (armStick.getRawButton(BUTTON_ARM_SIDE_PEG1)) {
            arm.goToPeg(true, false, scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), downFactor);
            Timer.delay(1);
        } else if (armStick.getRawButton(BUTTON_ARM_SIDE_PEG2)) {
             arm.goToPeg(false, false, scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), downFactor);
            Timer.delay(1);
        } else if (armStick.getRawButton(BUTTON_ARM_CARRY)) {
            arm.goToCarryPosition(scaleToRange(ds.getAnalogIn(AI_ARMSPEED) / 205, 0, 5, 0, 1), downFactor);
            Timer.delay(1);
        } else {
            if (Math.abs(armStick.getX() + armStick.getY()) < 0.31) {
                arm.handleJoystick(armStick, scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1), true);
            } else {
                arm.handleJoystick(armStick, scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1), false);
            }
        }
    }

    private void handleCameraMovement() {
        if (!armStick.getTrigger()) {
            if (armStick.getRawButton(BUTTON_CAMERA_UP)) {
                cameraTilt.setAngle(cameraTilt.getAngle() + CAMERA_INCREMENT_TILT);
            } else if (armStick.getRawButton(BUTTON_CAMERA_DOWN)) {
                cameraTilt.setAngle(cameraTilt.getAngle() - CAMERA_INCREMENT_TILT);
            }
            if (armStick.getRawButton(BUTTON_CAMERA_LEFT)) {
                cameraPan.setAngle(cameraPan.getAngle() - CAMERA_INCREMENT_PAN);
            } else if (armStick.getRawButton(BUTTON_CAMERA_RIGHT)) {
                cameraPan.setAngle(cameraPan.getAngle() + CAMERA_INCREMENT_PAN);
            }
            //handle reset
            if (armStick.getRawButton(BUTTON_CAMERA_RESET)) {
                cameraTilt.setAngle(DEFAULT_CAMERA_TILT);
                cameraPan.setAngle(DEFAULT_CAMERA_PAN);
            }
        }
    }

    private void handleClawMovement() { //needs work for new design
        if (armStick.getTrigger(Hand.kLeft)) {
            if (armStick.getRawButton(BUTTON_CLAW_TRIGGER_OPEN)) {
                claw.set(SPEED_CLAW_OPEN);
            } else if (armStick.getRawButton(BUTTON_CLAW_TRIGGER_CLOSE)) {
                claw.set(SPEED_CLAW_CLOSE);
            } else {
                claw.set(0);
            }
        } else {
            claw.set(0);
        }
        claw.Feed();
    }

    private void feedOutput() {
        //send camera feed
        camera = AxisCamera.getInstance();
        camera.writeResolution(AxisCamera.ResolutionT.k640x480);
        camera.writeBrightness(100);
        //update Dashboard
        updateDashboard();
        //update DO
        ds.setDigitalOut(DO_LEFTLINE, leftLineSensor.get());
        ds.setDigitalOut(DO_CENTERLINE, centerLineSensor.get());
        ds.setDigitalOut(DO_RIGHTLINE, rightLineSensor.get());
    }

    private void processInput() {
        startingAtFork = ds.getDigitalIn(DI_PATH);
        goRightAtFork = ds.getDigitalIn(DI_FORKDIR);
        useSpeedReg = ds.getDigitalIn(DI_SPEEDREG);
        //set arm power
        armPower = scaleToRange(-1.0 * armStick.getZ(), -1, 1, RANGE_POWER_ARM_LOW, RANGE_POWER_ARM_HIGH); //polarity is reversed on Z, so *-1
        arm.setMaxOutput(armPower);
        //set robot power
        robotPower = scaleToRange(-1.0 * rightStick.getZ(), -1, 1, RANGE_POWER_DRIVE_LOW, RANGE_POWER_DRIVE_HIGH); //polarity is reversed on Z, so *-1
        robotDrive.setMaxOutput(robotPower);
        //set down factor
        downFactor = scaleToRange(ds.getAnalogIn(AI_DOWNFACTOR) / 205, 0, 5, 0, 1);
        //set turn sensitivity
        if (rightStick.getRawButton(BUTTON_ROBOT_SENS_INC)) {
            if (sensitivityOverride == false) {
                sensitivityOverride = true;
                overriddenSensitivity = ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5) + sensitivityIncrement;
                oldSensitivity = ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5);
            } else {
                overriddenSensitivity += sensitivityIncrement;
            }
        } else if (rightStick.getRawButton(BUTTON_ROBOT_SENS_DEC)) {
            if (sensitivityOverride == false) {
                sensitivityOverride = true;
                overriddenSensitivity = ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5) - sensitivityIncrement;
                oldSensitivity = ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5);
            } else {
                overriddenSensitivity -= sensitivityIncrement;
            }
        } else if (rightStick.getRawButton(BUTTON_ROBOT_SENS_SCORE)) {
            overriddenSensitivity = SENS_PRESET_SCORE;
            sensitivityOverride = true;
            oldSensitivity = ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5);
        } else if (rightStick.getRawButton(BUTTON_ROBOT_SENS_TURN)) {
            overriddenSensitivity = SENS_PRESET_TURN;
            sensitivityOverride = true;
            oldSensitivity = ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5);
        }
        if (sensitivityOverride == true && Math.abs(oldSensitivity - ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5)) < 0.03) {
            robotSensitivity = overriddenSensitivity;
        } else {
            robotSensitivity = ((ds.getAnalogIn(AI_SENSITIVITY) / 205) / 5);
            sensitivityOverride = false;
        }
    }

    private void updateDashboard() {
        Dashboard lowDashData = DriverStation.getInstance().getDashboardPackerLow();
        lowDashData.addCluster();
        {
            lowDashData.addCluster();
            {     //analog modules
                lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++) {
                        lowDashData.addFloat((float) AnalogModule.getInstance(1).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();
                lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++) {
                        lowDashData.addFloat((float) AnalogModule.getInstance(2).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();
            }
            lowDashData.finalizeCluster();

            lowDashData.addCluster();
            { //digital modules
                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 4;
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 6;
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayReverse());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

            }
            lowDashData.finalizeCluster();

            lowDashData.addByte(Solenoid.getAll());
        }
        lowDashData.finalizeCluster();
        lowDashData.commit();
    }

    private void moveArmToPeg(boolean bottomPeg, boolean isCenterColumn) {
        arm.goToPeg(bottomPeg, isCenterColumn, ds.getAnalogIn(AI_ARMSPEED) / 205 / 5, downFactor);
    }

    private void goToScoreGrid() {
        double straightStopTime = AUTONOMOUS_TIME_STOP_STRAIGHT;
        double forkStopTime = AUTONOMOUS_TIME_STOP_FORK;
        double forkTime = AUTONOMOUS_TIME_FORK;
        double forkTimeProfile[] = AUTONOMOUS_PROFILE_TIME_FORK;
        double straightTimeProfile[] = AUTONOMOUS_PROFILE_TIME_STRAIGHT;
        double[][] forkDistProfile = AUTONOMOUS_PROFILE_DIST_FORK;
        double[][] straightDistProfile = AUTONOMOUS_PROFILE_DIST_STRAIGHT;
        double forkSpeed = AUTONOMOUS_SPEED_FORK;
        double forkDistance = AUTONOMOUS_DIST_FORK;
        boolean useDistanceMethod = AUTONOMOUS_USE_DIST_METHOD;
        double Kp = AUTONOMOUS_Kp;

        double defaultSteeringGain = ds.getAnalogIn(AI_STEERGAIN) / 205; // the default value for the steering gain
        int binaryValue = -1;
        int previousValue = 0;
        double steeringGain;
        double straightStartSpeed = straightTimeProfile[0];
        double straightEndSpeed = straightTimeProfile[straightTimeProfile.length - 1];
        double powerProfile[];
        boolean straightLine = ds.getDigitalIn(DI_PATH);
        double stopTime = (straightLine ? straightStopTime : forkStopTime); // when the robot should look for end
        boolean goLeft = !ds.getDigitalIn(DI_FORKDIR);

        Timer timer = new Timer();
        double time;
        double speed;
        double turn;

        int leftValue = leftLineSensor.get() ? 1 : 0;
        int middleValue = centerLineSensor.get() ? 1 : 0;
        int rightValue = rightLineSensor.get() ? 1 : 0;

        rightDriveEncoder.start();

        if (straightLine) {
            //use gyro and profiles
            powerProfile = straightTimeProfile;
            timer.start();
            timer.reset();
            while ((time = timer.get()) < stopTime && binaryValue != 7) {
                int timeInSeconds = (int) time;
                double timeInHalfSeconds;
                if (time + 0.5 < timeInSeconds + 1) {
                    timeInHalfSeconds = time;
                } else {
                    timeInHalfSeconds = time + 0.5;
                }
                if (goLeft) {
                    binaryValue = leftValue * 4 + middleValue * 2 + rightValue;
                    steeringGain = defaultSteeringGain;
                } else {
                    binaryValue = rightValue * 4 + middleValue * 2 + leftValue;
                    steeringGain = -defaultSteeringGain;
                }
                Watchdog.getInstance().feed();
                double angle = robotGyro.getAngle();
                if (!useDistanceMethod) {
                    robotDrive.drive(-powerProfile[(int)timeInHalfSeconds * 2], -angle * Kp);
                } else {
                    robotDrive.drive(-getSpeedFromDistance(rightDriveEncoder.getDistance(), false, forkDistProfile, straightDistProfile), -angle * Kp);
                }
                Timer.delay(0.004);
            }
        } else {
            powerProfile = forkTimeProfile;

            boolean atCross = false;
            timer.start();
            timer.reset();

            while ((time = timer.get()) <= stopTime && !atCross) {
                Watchdog.getInstance().feed();
                int timeInSeconds = (int) time;
                double timeInHalfSeconds;
                if (time + 0.5 < timeInSeconds + 1) {
                    timeInHalfSeconds = time;
                } else {
                    timeInHalfSeconds = time + 0.5;
                }
                if (goLeft) {
                    binaryValue = leftValue * 4 + middleValue * 2 + rightValue;
                    steeringGain = defaultSteeringGain;
                } else {
                    binaryValue = rightValue * 4 + middleValue * 2 + leftValue;
                    steeringGain = -defaultSteeringGain;
                }

                if (!useDistanceMethod) {
                    speed = -powerProfile[(int)(timeInHalfSeconds * 2)];
                } else {
                    speed = -getSpeedFromDistance(rightDriveEncoder.getDistance(), true, forkDistProfile, straightDistProfile);
                }
                if (rightDriveEncoder.getDistance() < forkDistance) {
                    if (Math.abs(speed) < Math.abs(straightStartSpeed)) {
                        speed = -straightStartSpeed;
                    }
                }
                turn = 0;

                switch (binaryValue) {
                    case 1:  // on line edge
                        turn = 0;
                        break;

                    case 7:  // all sensors on (maybe at cross)
                        if (time > forkTime) {
                            atCross = true;
                            speed = 0;
                        }
                    break;

                    case 0:  // all sensors off
                        if (previousValue == 0 || previousValue == 1) {
                            turn = steeringGain;
                        } else {
                            turn = -steeringGain;
                        }
                        break;

                    default:  // all other cases
                        turn = -steeringGain;
                }

                // set the robot speed and direction
                robotDrive.arcadeDrive(speed, turn);

                if (binaryValue != 0) {
                    previousValue = binaryValue;
                }

                Timer.delay(0.01);
            }

            robotDrive.arcadeDrive(0, 0);
            rightDriveEncoder.stop();
            rightDriveEncoder.reset();
        }
    }

    private double scaleToRange(double number, double initialRangeStart, double initialRangeEnd, double finalRangeStart, double finalRangeEnd) {
        double y = ((-1.0 * initialRangeStart) + initialRangeEnd) / (initialRangeEnd - finalRangeStart);
        double x = (finalRangeStart * y)-initialRangeStart;
        return (number + x) / y;
    }

    private double cubicSpeedReg(double input) {
        if (useSpeedReg) {
            return 0.1800*(input * input * input) + (0.8148 * input);
        } else {
            return input;
        }
    }

    private double getSpeedFromDistance(double distance, boolean forkPath, double[][] forkProfile, double[][] straightProfile) {
        double[][] profile = (forkPath ? forkProfile : straightProfile);

        for (int i = 0; i < profile.length; i++) {
            double[] set = profile[i];
            if (distance < set[0]) {
                return set[1];
            }
        }
        return profile[profile.length - 1][1];
    }

}