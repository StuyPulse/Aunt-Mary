/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ArmImpl extends Arm {

    private TalonFX armMotor;
    private CANcoder armEncoder;

    private Rotation2d targetAngle;

    private boolean rotateOverElevator;

    public ArmImpl() {

        armMotor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(armMotor);

        armEncoder = new CANcoder(Ports.Arm.ABSOLUTE_ENCODER);

        MagnetSensorConfigs magnet_config = new MagnetSensorConfigs()
            .withMagnetOffset(Constants.Arm.ANGLE_OFFSET)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        armEncoder.getConfigurator().apply(magnet_config);

        targetAngle = Rotation2d.fromDegrees(0.0);

        rotateOverElevator = false;
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public Rotation2d getArmAngle() {
        return Rotation2d.fromRotations(armMotor.getPosition().getValueAsDouble());
    }

    public boolean getRotateBoolean() {
        return rotateOverElevator;
    }

    public void setRotateBoolean(boolean overElevator) {
        rotateOverElevator = overElevator;
    }

    public boolean atTargetAngle() {
        return Math.abs(getArmAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.Arm.ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public void periodic() {
        
        // check?
        if (!getRotateBoolean()) {
            MotionMagicVoltage armOutput = new MotionMagicVoltage(getTargetAngle().getRotations());
            armMotor.setControl(armOutput);
        } else {
            MotionMagicVoltage armOutput = new MotionMagicVoltage(getTargetAngle().getRotations()-1);
            armMotor.setControl(armOutput);
        }
        
        SmartDashboard.putNumber("Arm/targetAngle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Arm/currentAngle", getArmAngle().getDegrees());
    }
}
