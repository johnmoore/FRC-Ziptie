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

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.parsing.IUtility;
import edu.wpi.first.wpilibj.*;

public class RobotArm implements MotorSafety, IUtility {

    protected MotorSafetyHelper m_safetyHelper;

    public static class MotorType {
        public final int value;
        static final int kArm_val = 1;

        public static final MotorType kArm = new MotorType(kArm_val);

        private MotorType(int value) {
            this.value = value;
        }
    }

    public static final double kDefaultExpirationTime = 0.1;
    public static final double kDefaultSensitivity = 0.5;
    public static final double kDefaultMaxOutput = 1.0;
    protected static final int kMaxNumberOfMotors = 4;
    protected final int m_invertedMotors[] = new int[4];
    protected double m_sensitivity;
    protected double m_maxOutput;
    protected SpeedController m_armMotor;
    protected boolean m_allocatedSpeedControllers;

    public RobotArm(final int armMotorChannel) {
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        m_armMotor = new Jaguar(armMotorChannel);
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
            m_invertedMotors[i] = 1;
        }
        setupMotorSafety();
        drive(0, 0);
    }

    public void drive(double outputMagnitude, double curve) {
        double leftOutput, rightOutput;

        if (curve < 0) {
            double value = MathUtils.log(-curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude / ratio;
            rightOutput = outputMagnitude;
        } else if (curve > 0) {
            double value = MathUtils.log(curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude / ratio;
        } else {
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude;
        }
        setLeftRightMotorOutputs(leftOutput, rightOutput, 1.0);
    }

    public void arcadeDrive(GenericHID stick, boolean squaredInputs, double downFactor) {
        // simply call the full-featured arcadeDrive with the appropriate values
        arcadeDrive(stick.getY(), stick.getX(), squaredInputs, downFactor);
    }

    public void arcadeDrive(GenericHID stick, double downFactor) {
        this.arcadeDrive(stick, true, downFactor);
    }

    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis,
            boolean squaredInputs, double downFactor) {
        double moveValue = moveStick.getRawAxis(moveAxis);
        double rotateValue = rotateStick.getRawAxis(rotateAxis);

        arcadeDrive(moveValue, rotateValue, squaredInputs, downFactor);
    }

    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis, double downFactor) {
        this.arcadeDrive(moveStick, moveAxis, rotateStick, rotateAxis, true, downFactor);
    }

    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs, double downFactor) {
        // local variables to hold the computed PWM values for the motors
        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed, downFactor);
    }

    public void arcadeDrive(double moveValue, double rotateValue, double downFactor) {
        this.arcadeDrive(moveValue, rotateValue, true, downFactor);
    }

    public void setLeftRightMotorOutputs(double leftOutput, double rightOutput, double downFactor) {
        if (m_armMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte)0x80;
        double outputSpeed = limit(leftOutput + rightOutput);

        if (outputSpeed < 0) {
          outputSpeed = outputSpeed * downFactor;
        }

            m_armMotor.set(limit(leftOutput + rightOutput) * m_invertedMotors[MotorType.kArm_val] * m_maxOutput, syncGroup);


        try {
	     CANJaguar.updateSyncGroup(syncGroup);
	} catch(Exception e) {

	}

        if (m_safetyHelper != null) m_safetyHelper.feed();
    }

    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    protected static void normalize(double wheelSpeeds[]) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        int i;
        for (i=1; i<kMaxNumberOfMotors; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) maxMagnitude = temp;
        }
        if (maxMagnitude > 1.0) {
            for (i=0; i<kMaxNumberOfMotors; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

    protected static double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle * (3.14159 / 180.0));
        double sinA = Math.sin(angle * (3.14159 / 180.0));
        double out[] = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }

    public void setInvertedMotor(MotorType motor, boolean isInverted) {
        m_invertedMotors[motor.value] = isInverted ? -1 : 1;
    }

    public void setSensitivity(double sensitivity)
    {
            m_sensitivity = sensitivity;
    }

    public void setMaxOutput(double maxOutput)
    {
            m_maxOutput = maxOutput;
    }

    public void setExpiration(double timeout) {
        m_safetyHelper.setExpiration(timeout);
    }

    public double getExpiration() {
        return m_safetyHelper.getExpiration();
    }

    public boolean isAlive() {
        return m_safetyHelper.isAlive();
    }

    public boolean isSafetyEnabled() {
        return m_safetyHelper.isSafetyEnabled();
    }

    public void setSafetyEnabled(boolean enabled) {
        m_safetyHelper.setSafetyEnabled(enabled);
    }

    public void stopMotor() {
        if (m_armMotor != null) {
            m_armMotor.set(0.0);
        }
    }

    private void setupMotorSafety() {
        m_allocatedSpeedControllers = true;
        m_safetyHelper = new MotorSafetyHelper(this);
        m_safetyHelper.setExpiration(kDefaultExpirationTime);
        m_safetyHelper.setSafetyEnabled(true);
    }
}