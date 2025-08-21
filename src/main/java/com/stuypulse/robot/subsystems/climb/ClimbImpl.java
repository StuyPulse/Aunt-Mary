
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbImpl extends Climb {
    private TalonFX motor;
    private DutyCycleEncoder absoluteEncoder;

    protected ClimbImpl() {
        super();
        motor = new TalonFX(Ports.Climb.MOTOR, "can_s3");
        Motors.Climb.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Settings.Climb.OPEN_ANGLE_DEG/360.0);

        absoluteEncoder = new DutyCycleEncoder(Ports.Climb.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(false);
    }

    private double getTargetAngleDeg() {
        return getState().getTargetAngle();
    }
        
    @Override
    public double getCurrentAngleDeg() {
        double angle = 360.0*((absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET_DEG/360.0));

        //while (angle/360.0 < (Constants.Climb.MIN_ANGLE_DEG-110.0)/360.0) {
            //angle = 360.0 + angle;
        //}

        return angle;
    }

    // public Rotation2d getCurrentAngle() {
    //     return absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET.getRotations() < Constants.Climb.MIN_ANGLE.minus(Rotation2d.fromDegrees(10)).getRotations()
    //         ? Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET.getRotations() + 1)
    //         : Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET.getRotations());
    // }

    @Override
    public void periodic() {
        super.periodic();

        if (Settings.EnabledSubsystems.CLIMB.get()) {
            double angleErrorDegrees = getTargetAngleDeg() - getCurrentAngleDeg();

            if (getState() == ClimbState.IDLE) {
                motor.setVoltage(0);
            }
            else if (getState() == ClimbState.OPEN) {
                if (angleErrorDegrees < 0) {
                    if (Math.abs(angleErrorDegrees) < 15) {
                        motor.setVoltage(-Settings.Climb.OPEN_VOLTAGE_LOW);
                    }
                    else {
                        motor.setVoltage(-Settings.Climb.DEFAULT_VOLTAGE);
                    }
                }
                else {
                    motor.setVoltage(0);
                }
            }
            else if (getState() == ClimbState.CLOSED) {
                if (Math.abs(angleErrorDegrees) > Settings.Climb.ANGLE_TOLERANCE_FOR_CLOSED_AND_SHIMMY_DEG) {
                    if (getCurrentAngleDeg() > Settings.Climb.CLOSED_ANGLE_DEG) {
                        motor.setVoltage(-Settings.Climb.DEFAULT_VOLTAGE);
                    }
                    else {
                        motor.setVoltage(Settings.Climb.DEFAULT_VOLTAGE);
                    }
                }
                else {
                    motor.setVoltage(0);
                }
            }
            else if (getState() == ClimbState.SHIMMY) {
                if (Math.abs(angleErrorDegrees) > Settings.Climb.ANGLE_TOLERANCE_FOR_CLOSED_AND_SHIMMY_DEG) {
                    if (getCurrentAngleDeg() > Settings.Climb.SHIMMY_ANGLE_DEG) {
                        motor.setVoltage(-Settings.Climb.DEFAULT_VOLTAGE);
                    }
                    else {
                        motor.setVoltage(Settings.Climb.DEFAULT_VOLTAGE);
                    }
                }
                else {
                    motor.setVoltage(0);
                }
            }
            else if (getState() == ClimbState.CLIMBING) {
                if (getCurrentAngleDeg() < Settings.Climb.CLIMBED_ANGLE_DEG) {
                    motor.setVoltage(Settings.Climb.CLIMB_VOLTAGE);
                }
                else {
                    motor.setVoltage(0);
                }
            }
        }
        else {
            motor.setVoltage(0);
        }

        SmartDashboard.putNumber("Climb/Current Angle (deg)", getCurrentAngleDeg());
        SmartDashboard.putNumber("Climb/Target Angle (deg)", getTargetAngleDeg());

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Climb/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Climb/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Climb/Stator Current", motor.getStatorCurrent().getValueAsDouble());
        }
    }
}
