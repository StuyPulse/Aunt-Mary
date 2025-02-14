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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class FroggyImpl extends Froggy {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    // private CANcoder absoluteEncoder;
    private DutyCycleEncoder absoluteEncoder;

    private BStream isStalling;

    public FroggyImpl() {
        rollerMotor = new TalonFX(Ports.Froggy.ROLLER);
        Motors.Froggy.ROLLER_MOTOR_CONFIG.configure(rollerMotor);

        pivotMotor = new TalonFX(Ports.Froggy.PIVOT);
        Motors.Froggy.PIVOT_MOTOR_CONFIG.configure(pivotMotor);

        // absoluteEncoder = new CANcoder(Ports.Froggy.ABSOLUTE_ENCODER);

        // MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
        //     .withMagnetOffset(Constants.Froggy.ANGLE_OFFSET)
        //     .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        // absoluteEncoder.getConfigurator().apply(magnetSensorConfigs);
       
        absoluteEncoder = new DutyCycleEncoder(Ports.Froggy.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(true);

        isStalling = BStream.create(() -> {
            switch (getRollerState()) {
                case INTAKE_CORAL:
                    return Math.abs(rollerMotor.getSupplyCurrent().getValueAsDouble()) > Settings.Froggy.CORAL_STALL_CURRENT_THRESHOLD;
                case INTAKE_ALGAE:
                    return Math.abs(rollerMotor.getSupplyCurrent().getValueAsDouble()) > Settings.Froggy.ALGAE_STALL_CURRENT_THRESHOLD;
                default:
                    return false;
            }
        }).filtered(new BDebounce.Both(Settings.Froggy.STALL_DEBOUNCE_TIME));
    }

    private void setRollerBasedOnState() {
        switch (getRollerState()) {
            case INTAKE_CORAL:
                rollerMotor.set(-Settings.Froggy.CORAL_INTAKE_SPEED);
                break;
            case INTAKE_ALGAE:
                rollerMotor.set(Settings.Froggy.ALGAE_INTAKE_SPEED);
                break;
            case SHOOT_ALGAE:
                rollerMotor.set(-Settings.Froggy.ALGAE_OUTTAKE_SPEED);
                break;
            case SHOOT_CORAL:
                rollerMotor.set(Settings.Froggy.CORAL_OUTTAKE_SPEED);
                break;
            case HOLD_ALGAE:
                rollerMotor.set(Settings.Froggy.HOLD_ALGAE_SPEED);
                break;
            case STOP:
                rollerMotor.set(0);
            default:
                break;
        }
    }

    private Rotation2d getCurrentAngle() {
        // return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        return Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Froggy.ANGLE_OFFSET.getRotations());
    }

    private Rotation2d getTargetAngle() {
        Rotation2d targetAngle;
        switch (getPivotState()) {
            case STOW:
                targetAngle = Settings.Froggy.STOW_ANGLE;
            case ALGAE_GROUND_PICKUP:
                targetAngle = Settings.Froggy.ALGAE_GROUND_PICKUP_ANGLE;
            case CORAL_GROUND_PICKUP:
                targetAngle = Settings.Froggy.CORAL_GROUND_PICKUP_ANGLE;
            case GOLF_TEE_ALGAE_PICKUP:
                targetAngle = Settings.Froggy.GOLF_TEE_ALGAE_PICKUP_ANGLE;
            case L1_SCORE_ANGLE:
                targetAngle = Settings.Froggy.L1_SCORING_ANGLE;
            case PROCESSOR_SCORE_ANGLE:
                targetAngle = Settings.Froggy.PROCESSOR_SCORE_ANGLE;
            default:
                targetAngle = Settings.Froggy.STOW_ANGLE;
        }
        targetAngle = Rotation2d.fromDegrees(SLMath.clamp(targetAngle.getDegrees(), Constants.Froggy.MINIMUM_ANGLE.getDegrees(), Constants.Froggy.MAXIMUM_ANGLE.getDegrees()));
        return targetAngle;
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(getTargetAngle().getDegrees() - getCurrentAngle().getDegrees()) < Settings.Froggy.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isStalling() {
        return isStalling.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        setRollerBasedOnState();

        MotionMagicVoltage pivotControlRequest = new MotionMagicVoltage(getTargetAngle().getRotations());
        pivotMotor.setControl(pivotControlRequest);

        SmartDashboard.putBoolean("Froggy/At Target Angle", isAtTargetAngle());

        SmartDashboard.putNumber("Froggy/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putBoolean("Froggy/Is Stalling", isStalling());
        
        SmartDashboard.putNumber("Froggy/Roller Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
    }
}
