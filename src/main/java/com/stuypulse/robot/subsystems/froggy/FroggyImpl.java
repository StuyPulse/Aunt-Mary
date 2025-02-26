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

    private BStream isStalling;

    private Controller controller;
    private MotionProfile motionProfile;
    private SettableNumber kP, kI, kD, kS, kV, kA, kG;

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

        kP = new SettableNumber(Gains.Froggy.Coral.PID.kP);
        kI = new SettableNumber(Gains.Froggy.Coral.PID.kI);
        kD = new SettableNumber(Gains.Froggy.Coral.PID.kD);
        kS = new SettableNumber(Gains.Froggy.Coral.FF.kS);
        kV = new SettableNumber(Gains.Froggy.Coral.FF.kV);
        kA = new SettableNumber(Gains.Froggy.Coral.FF.kA);
        kG = new SettableNumber(Gains.Froggy.Coral.FF.kG);

        motionProfile = new MotionProfile(Settings.Froggy.MAX_VEL.getDegrees(), Settings.Froggy.MAX_ACCEL.getDegrees());

        controller = new MotorFeedforward(kS, kV, kA).position()
            .add(new ArmFeedforward(kG))
            .add(new PIDController(kP, kI, kD))
            .setSetpointFilter(motionProfile);
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

    private void updateGains() {
        if (getRollerState() == RollerState.HOLD_ALGAE && isStalling()) {
            kP.set(Gains.Froggy.Algae.PID.kP);
            kI.set(Gains.Froggy.Algae.PID.kI);
            kD.set(Gains.Froggy.Algae.PID.kD);
            kS.set(Gains.Froggy.Algae.FF.kS);
            kV.set(Gains.Froggy.Algae.FF.kV);
            kA.set(Gains.Froggy.Algae.FF.kA);
            kG.set(Gains.Froggy.Algae.FF.kG);
        } else {
            kP.set(Gains.Froggy.Coral.PID.kP);
            kI.set(Gains.Froggy.Coral.PID.kI);
            kD.set(Gains.Froggy.Coral.PID.kD);
            kS.set(Gains.Froggy.Coral.FF.kS);
            kV.set(Gains.Froggy.Coral.FF.kV);
            kA.set(Gains.Froggy.Coral.FF.kA);
            kG.set(Gains.Froggy.Coral.FF.kG);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        updateGains();

        pivotMotor.setPosition(getCurrentAngle().getRotations());

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

        // ROLLER
        SmartDashboard.putBoolean("Froggy/Roller/Is Stalling", isStalling());
        
        SmartDashboard.putNumber("Froggy/Roller/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller/Supply Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Froggy/Roller/Stator Current", rollerMotor.getStatorCurrent().getValueAsDouble());
    }
}
