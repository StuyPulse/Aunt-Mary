/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

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

    private Controller controller;

    private BStream isStalling;

    private Optional<Double> pivotVoltageOverride;
    private Rotation2d pivotOperatorOffset;

    protected FroggyImpl() {
        super();
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

        controller = new MotorFeedforward(Gains.Froggy.FF.kS, Gains.Froggy.FF.kV, Gains.Froggy.FF.kA).position()
            .add(new ArmFeedforward(Gains.Froggy.FF.kG))
            .add(new PIDController(Gains.Froggy.PID.kP, Gains.Froggy.PID.kI, Gains.Froggy.PID.kD))
            .setSetpointFilter(new MotionProfile(Settings.Froggy.MAX_VEL_ROTATIONS_PER_S, Settings.Froggy.MAX_ACCEL_ROTATIONS_PER_S_PER_S));

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

        pivotVoltageOverride = Optional.empty();
        pivotOperatorOffset = Rotation2d.kZero;
    }

    private Rotation2d getCurrentAngle() {
        // return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        return Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Froggy.ANGLE_OFFSET.getRotations());
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(SLMath.clamp(getPivotState().getTargetAngle().getDegrees(), Constants.Froggy.MINIMUM_ANGLE.getDegrees(), Constants.Froggy.MAXIMUM_ANGLE.getDegrees()))
            .plus(pivotOperatorOffset);
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
    public void setPivotVoltageOverride(Optional<Double> voltage) {
        this.pivotVoltageOverride = voltage;
    }

    @Override
    public void setPivotOperatorOffset(Rotation2d offset) {
        this.pivotOperatorOffset = offset;
    }

    @Override
    public Rotation2d getPivotOperatorOffset() {
        return this.pivotOperatorOffset;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (Settings.EnabledSubsystems.FROGGY.get()) {
            rollerMotor.set(getRollerState().getTargetSpeed().doubleValue());
            if (pivotVoltageOverride.isPresent()) {
                pivotMotor.setVoltage(pivotVoltageOverride.get());
            }
            else {
                // pivotMotor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations()));
                pivotMotor.setVoltage(controller.update(getTargetAngle().getRotations(), getCurrentAngle().getRotations()));
            }
        }
        else {
            rollerMotor.set(0);
            pivotMotor.setVoltage(0);
        }

        SmartDashboard.putBoolean("Froggy/At Target Angle", isAtTargetAngle());

        SmartDashboard.putNumber("Froggy/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putBoolean("Froggy/Is Stalling", isStalling());
        
        SmartDashboard.putNumber("Froggy/Roller Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
    }
}
