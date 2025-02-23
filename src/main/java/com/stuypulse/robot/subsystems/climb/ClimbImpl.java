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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbImpl extends Climb {
    private TalonFX motor;
    private DutyCycleEncoder absoluteEncoder;

    private Optional<Double> voltageOverride;

    protected ClimbImpl() {
        super();
        motor = new TalonFX(Ports.Climb.MOTOR);
        Motors.Climb.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Settings.Climb.OPEN_ANGLE.getRotations());

        absoluteEncoder = new DutyCycleEncoder(Ports.Climb.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(false);

        voltageOverride = Optional.empty();
    }

    private Rotation2d getTargetAngle() {
        return getState().getTargetAngle();
    }

    private Rotation2d getCurrentAngle() {
        return absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET.getRotations() < Constants.Climb.MIN_ANGLE.minus(Rotation2d.fromDegrees(10)).getRotations()
            ? Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET.getRotations() + 1)
            : Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET.getRotations());
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (Settings.EnabledSubsystems.CLIMB.get()) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            }
            else {
                double angleErrorDegrees = getTargetAngle().getDegrees() - getCurrentAngle().getDegrees();
                SmartDashboard.putNumber("Climb/Angle Error (deg)", angleErrorDegrees);

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
                    if (Math.abs(angleErrorDegrees) > Settings.Climb.ANGLE_TOLERANCE_FOR_CLOSED.getDegrees()) {
                        if (getCurrentAngle().getDegrees() > Settings.Climb.CLOSED_ANGLE.getDegrees()) {
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
                    if (getCurrentAngle().getDegrees() < Settings.Climb.CLIMBED_ANGLE.getDegrees()) {
                        motor.setVoltage(Settings.Climb.CLIMB_VOLTAGE);
                    }
                    else {
                        motor.setVoltage(0);
                    }
                }
            }
        }
        else {
            motor.setVoltage(0);
        }

        SmartDashboard.putNumber("Climb/Absolute Encoder angle raw (deg)", Units.rotationsToDegrees(absoluteEncoder.get()));
        SmartDashboard.putNumber("Climb/Absolute Encoder angle (deg)", Units.rotationsToDegrees(absoluteEncoder.get() - Constants.Climb.ANGLE_OFFSET.getRotations()));

        SmartDashboard.putNumber("Climb/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Climb/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Climb/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
