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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbImpl extends Climb {
    private final TalonFX motor;
    private final CANcoder absoluteEncoder;

    public ClimbImpl() {
        motor = new TalonFX(Ports.Climb.MOTOR);
        Motors.Climb.MOTOR_CONFIG.configure(motor);

        absoluteEncoder = new CANcoder(Ports.Climb.ABSOLUTE_ENCODER);

        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
            .withMagnetOffset(Constants.Climb.ANGLE_OFFSET.getRotations());

        absoluteEncoder.getConfigurator().apply(magnetSensorConfigs);
    }

    private Rotation2d getTargetAngle() {
        switch (getState()) {
            case STOW:
                return Settings.Climb.STOW_ANGLE;
            case OPEN:
                return Settings.Climb.OPEN_ANGLE;
            case CLIMBING:
                return Settings.Climb.CLIMBED_ANGLE;
            default:
                return Settings.Climb.STOW_ANGLE;
        }
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getTargetAngle().getDegrees() - getCurrentAngle().getDegrees()) < Settings.Climb.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public void periodic() {
        motor.setControl(new PositionVoltage(getTargetAngle().getRotations()));

        SmartDashboard.putNumber("Climb/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Climb/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Climb/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
