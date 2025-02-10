/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class FroggyImpl extends Froggy {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private CANcoder absoluteEncoder;

    private final BStream isCoralStalling;
    private final BStream isAlgaeStalling;

    public Rotation2d targetAngle;

    public FroggyImpl() {
        rollerMotor = new TalonFX(Ports.Froggy.ROLLER);
        Motors.Froggy.ROLLER_MOTOR_CONFIG.configure(rollerMotor);

        pivotMotor = new TalonFX(Ports.Froggy.PIVOT);
        Motors.Froggy.PIVOT_MOTOR_CONFIG.configure(pivotMotor);

        absoluteEncoder = new CANcoder(Ports.Froggy.ABSOLUTE_ENCODER);

        MagnetSensorConfigs magnetSensorConfigs =
                new MagnetSensorConfigs()
                        .withMagnetOffset(Constants.Froggy.ANGLE_OFFSET)
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        absoluteEncoder.getConfigurator().apply(magnetSensorConfigs);

        targetAngle = Rotation2d.fromDegrees(Settings.Froggy.STOW_ANGLE.getDegrees());

        isCoralStalling =
                BStream.create(
                                () ->
                                        rollerMotor.getSupplyCurrent().getValueAsDouble()
                                                < -Settings.Froggy.CORAL_CURRENT_THRESHOLD) // coral stalls in the negative direction
                        .filtered(new BDebounce.Rising(Settings.Froggy.STALL_DEBOUNCE_TIME));

        isAlgaeStalling =
                BStream.create(
                                () ->
                                        rollerMotor.getSupplyCurrent().getValueAsDouble()
                                                > Settings.Froggy.ALGAE_CURRENT_THRESHOLD)
                        .filtered(new BDebounce.Rising(Settings.Froggy.STALL_DEBOUNCE_TIME));
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle =
                Rotation2d.fromDegrees(
                        SLMath.clamp(
                                targetAngle.getDegrees(),
                                Constants.Froggy.MINIMUM_ANGLE,
                                Constants.Froggy.MAXIMUM_ANGLE));
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(targetAngle.getDegrees() - getCurrentAngle().getDegrees())
                < Settings.Froggy.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public void intakeAlgae() {
        rollerMotor.set(Settings.Froggy.ALGAE_INTAKE_SPEED);
    }

    @Override
    public void outakeAlgae() {
        rollerMotor.set(-Settings.Froggy.ALGAE_OUTTAKE_SPEED);
    }

    @Override
    public void intakeCoral() {
        rollerMotor.set(-Settings.Froggy.CORAL_INTAKE_SPEED);
    }

    public void outakeCoral() {
        rollerMotor.set(Settings.Froggy.CORAL_OUTTAKE_SPEED);
    }

    public void holdAlgae() {
        rollerMotor.set(Settings.Froggy.HOLD_ALGAE_SPEED);
    }

    @Override
    public boolean isAlgaeStalling() {
        return isAlgaeStalling.get();
    }

    @Override
    public boolean isCoralStalling() {
        return isCoralStalling.get();
    }

    @Override
    public void stopRoller() {
        rollerMotor.set(0);
    }

    @Override
    public void periodic() {
        MotionMagicVoltage controllerOutput = new MotionMagicVoltage(targetAngle.getRotations());
        rollerMotor.setControl(controllerOutput);

        SmartDashboard.putNumber("Froggy/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putBoolean("Froggy/Has Algae", isAlgaeStalling());
        SmartDashboard.putBoolean("Froggy/Has Coral", isCoralStalling());
        
        SmartDashboard.putNumber("Froggy/Roller Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
    }
}
