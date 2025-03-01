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
import com.stuypulse.robot.util.SettableNumber;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

public class FroggyImpl extends Froggy {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private DutyCycleEncoder absoluteEncoder;

    private BStream isStallingCoral;
    private BStream isStallingAlgae;

    private Controller controller;
    private MotionProfile motionProfile;

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

        isStallingCoral = BStream.create(() -> Math.abs(rollerMotor.getStatorCurrent().getValueAsDouble()) > Settings.Froggy.CORAL_STALL_CURRENT_THRESHOLD)
            .filtered(new BDebounce.Both(Settings.Froggy.STALL_DEBOUNCE_TIME));
        isStallingAlgae = BStream.create(() -> Math.abs(rollerMotor.getStatorCurrent().getValueAsDouble()) > Settings.Froggy.ALGAE_STALL_CURRENT_THRESHOLD)
            .filtered(new BDebounce.Both(Settings.Froggy.STALL_DEBOUNCE_TIME));

        pivotVoltageOverride = Optional.empty();
        pivotOperatorOffset = Rotation2d.kZero;

        motionProfile = new MotionProfile(Settings.Froggy.MAX_VEL.getDegrees(), Settings.Froggy.MAX_ACCEL.getDegrees());

        controller = new MotorFeedforward(Gains.Froggy.FF.kS, Gains.Froggy.FF.kV, Gains.Froggy.FF.kA).position()
            .add(new ArmFeedforward(Gains.Froggy.FF.kG))
            .add(new PIDController(Gains.Froggy.PID.kP, Gains.Froggy.PID.kI, Gains.Froggy.PID.kD))
            .setSetpointFilter(motionProfile);
    }

    @Override
    public SysIdRoutine getFroggySysIdRoutine() {
        return SysId.getRoutine(
            1, 
            5, 
            "Froggy Pivot", 
            voltage -> setPivotVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngle().getDegrees(), 
            () -> Units.rotationsToDegrees(pivotMotor.getVelocity().getValueAsDouble()), 
            () -> pivotMotor.getMotorVoltage().getValueAsDouble(), 
            getInstance()
        );
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Froggy.ANGLE_OFFSET.getRotations());
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(
                getPivotState().getTargetAngle().getDegrees() + pivotOperatorOffset.getDegrees(),
                Constants.Froggy.MINIMUM_ANGLE.getDegrees(),
                Constants.Froggy.MAXIMUM_ANGLE.getDegrees()));
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(getTargetAngle().getDegrees() - getCurrentAngle().getDegrees()) < Settings.Froggy.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isStallingCoral() {
        return isStallingCoral.get();
    }

    @Override
    public boolean isStallingAlgae() {
        return isStallingAlgae.get();
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
                pivotMotor.setVoltage(controller.update(getTargetAngle().getDegrees(), getCurrentAngle().getDegrees()));
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
        SmartDashboard.putNumber("Froggy/Pivot/Setpoint (deg)", controller.getSetpoint());

        // ROLLER
        SmartDashboard.putBoolean("Froggy/Roller/Is Stalling Coral", isStallingCoral());
        SmartDashboard.putBoolean("Froggy/Roller/Is Stalling Algae", isStallingAlgae());
        
        SmartDashboard.putNumber("Froggy/Roller/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller/Supply Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller/Stator Current", rollerMotor.getStatorCurrent().getValueAsDouble());
    }
}
