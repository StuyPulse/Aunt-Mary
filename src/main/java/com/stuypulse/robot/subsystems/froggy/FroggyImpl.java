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
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class FroggyImpl extends Froggy {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private DutyCycleEncoder absoluteEncoder;

    private BStream isStalling;

    private Optional<Double> pivotVoltageOverride;
    private Rotation2d pivotOperatorOffset;

    protected FroggyImpl() {
        super();
        rollerMotor = new TalonFX(Ports.Froggy.ROLLER);
        Motors.Froggy.ROLLER_MOTOR_CONFIG.configure(rollerMotor);

        pivotMotor = new TalonFX(Ports.Froggy.PIVOT);
        Motors.Froggy.PIVOT_MOTOR_CONFIG.configure(pivotMotor);
       
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

        pivotVoltageOverride = Optional.empty();
        pivotOperatorOffset = Rotation2d.kZero;
    }

    @Override
    public SysIdRoutine getFroggySysIdRoutine() {
        return SysId.getRoutine(
            2, 
            9, 
            "Froggy Pivot", 
            voltage -> setPivotVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngle().getRotations(), 
            () -> pivotMotor.getVelocity().getValueAsDouble(), 
            () -> pivotMotor.getMotorVoltage().getValueAsDouble(), 
            getInstance()
        );
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

        pivotMotor.setPosition(getCurrentAngle().getRotations());

        if (Settings.EnabledSubsystems.FROGGY.get()) {
            rollerMotor.set(getRollerState().getTargetSpeed().doubleValue());
            if (pivotVoltageOverride.isPresent()) {
                pivotMotor.setVoltage(pivotVoltageOverride.get());
            }
            else {
                if (getRollerState() == RollerState.HOLD_ALGAE && isStalling()) {
                    // Algae
                    pivotMotor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations()).withSlot(1));
                }
                else {
                    // Coral
                    pivotMotor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations()).withSlot(0));
                }
            }
        }
        else {
            rollerMotor.set(0);
            pivotMotor.setVoltage(0);
        }

        // PIVOT
        SmartDashboard.putBoolean("Froggy/Pivot/At Target Angle", isAtTargetAngle());
        SmartDashboard.putNumber("Froggy/Pivot/Raw Encoder Angle (deg)", Units.rotationsToDegrees(absoluteEncoder.get()));

        SmartDashboard.putNumber("Froggy/Pivot/Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Pivot/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Froggy/Pivot/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Pivot/Target Angle (deg)", getTargetAngle().getDegrees());

        // ROLLER
        SmartDashboard.putBoolean("Froggy/Roller/Is Stalling", isStalling());
        
        SmartDashboard.putNumber("Froggy/Roller/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller/Supply Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller/Stator Current", rollerMotor.getStatorCurrent().getValueAsDouble());
    }
}
