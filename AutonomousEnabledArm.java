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

import edu.wpi.first.wpilibj.*;

public class AutonomousEnabledArm extends RobotArm {

    private double armAngle = 0;

    final static int ARM_START_VAL = 0;
    final static int MIN_ARM_VAL = 97;
    final static int MAX_ARM_VAL = 475;


    final static double PEG_SIDE_BOTTOM_VAL = 340;
    final static double PEG_SIDE_MIDDLE_VAL = 340;
    final static double PEG_CENTER_BOTTOM_VAL = 370;
    final static double PEG_CENTER_MIDDLE_VAL = 370;
    final static int ARM_CARRY_VAL = 180;

    final static double downResistance = 0.31;
    final static double armAutoTimeout = 3;

    Encoder armEncoder;

    public AutonomousEnabledArm(final int armMotorChannel, final int armEncoderChannelA, final int armEncoderChannelB) {
        super(armMotorChannel);
        armEncoder = new Encoder(new DigitalInput(armEncoderChannelA), new DigitalInput(armEncoderChannelB), false, CounterBase.EncodingType.k4X);
    }

    public void startArmEncoder() {
        armEncoder.start();
    }
    public void resetArmEncoder() {
        armEncoder.reset();
    }

    public void goToPeg(boolean bottomPeg, boolean isCenterColumn, double speed, double downFactor) {
        double destVal = 0;
        if (bottomPeg && isCenterColumn) {
            destVal = PEG_CENTER_BOTTOM_VAL;
        } else if (bottomPeg && !isCenterColumn) {
            destVal = PEG_SIDE_BOTTOM_VAL;
        } else if (!bottomPeg && isCenterColumn) {
            destVal = PEG_CENTER_MIDDLE_VAL;
        } else {
            destVal = PEG_SIDE_MIDDLE_VAL;
        }
        goToValue(destVal, speed, downFactor);
    }

    public void goToCarryPosition(double speed, double downFactor) {
        this.goToValue(ARM_CARRY_VAL, speed, downFactor);
    }

    public void goToValue(double destination, double speed, double downFactor) {
        if (destination > this.getEncoderCount()) {
            Timer timer = new Timer();
            timer.start();
            while (destination > this.getEncoderCount() && timer.get() < armAutoTimeout) {
                this.raise(30, speed, downFactor);
            }
        } else {
            Timer timer = new Timer();
            timer.start();
            while (destination < this.getEncoderCount() && timer.get() < armAutoTimeout) {
                this.lower(30, speed, downFactor);
            }
        }
        setLeftRightMotorOutputs(0, 0, downFactor);
    }

    public void raise(double increment, double speed, double downFactor) {
        double startVal = armEncoder.getDistance();
        double destVal = startVal + increment;
        Timer timer = new Timer();
        timer.start();
        while (armEncoder.getDistance() < destVal && timer.get() < armAutoTimeout) {
            setLeftRightMotorOutputs(limit(speed) / 2, limit(speed) / 2, downFactor);
        }
    }

    public void lower(double increment, double speed, double downFactor) {
        double startVal = armEncoder.getDistance();
        double destVal = startVal - increment;
        Timer timer = new Timer();
        timer.start();
        while (armEncoder.getDistance() > destVal && timer.get() < armAutoTimeout) {
            setLeftRightMotorOutputs(-limit(speed) / 2, -limit(speed) / 2, downFactor * 2);
        }
    }

    public void handleJoystick(GenericHID stick, double downFactor, boolean stopDrop) {
        if (!stopDrop) {
            super.arcadeDrive(stick, downFactor);
        } else {
            super.arcadeDrive(downResistance, 0, true, downFactor);
        }
    }

    public void setMaxOutput(double maxOutput)
    {
            super.m_maxOutput = maxOutput;
    }

    public double getEncoderCount() {
        return armEncoder.getDistance();

    }
    public void setLeftRightMotorOutputs(double leftOutput, double rightOutput, double downFactor) {
        if (m_armMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte)0x80;
        double outputSpeed = limit(leftOutput + rightOutput);

        if (outputSpeed < 0) { //compensate for gravity pulling the arm down
          outputSpeed = outputSpeed * Math.abs(downFactor);
        }

        m_armMotor.set(outputSpeed * m_invertedMotors[MotorType.kArm_val] * m_maxOutput, syncGroup);

        try {
	     CANJaguar.updateSyncGroup(syncGroup);
	} catch(Exception e) {

	}

        if (m_safetyHelper != null) m_safetyHelper.feed();
    }
}