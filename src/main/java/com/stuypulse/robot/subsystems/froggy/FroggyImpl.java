
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;

public class FroggyImpl extends Froggy {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private DutyCycleEncoder absoluteEncoder;
    private boolean hasUsedAbsoluteEncoderToSetPivot;

    private MotionProfile debuggingMotionProfile;

    private Optional<Double> pivotVoltageOverride;

    protected FroggyImpl() {
        super();
        rollerMotor = new TalonFX(Ports.Froggy.ROLLER);
        Motors.Froggy.ROLLER_MOTOR_CONFIG.configure(rollerMotor);

        pivotMotor = new TalonFX(Ports.Froggy.PIVOT);
        Motors.Froggy.PIVOT_MOTOR_CONFIG.configure(pivotMotor);
        pivotMotor.setPosition(Constants.Froggy.MAXIMUM_ANGLE.getRotations());
       
        absoluteEncoder = new DutyCycleEncoder(Ports.Froggy.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(false);

        hasUsedAbsoluteEncoderToSetPivot = false;

        pivotVoltageOverride = Optional.empty();

        debuggingMotionProfile = new MotionProfile(Settings.Froggy.MAX_VEL.getDegrees(), Settings.Froggy.MAX_ACCEL.getDegrees());
    }

    @Override
    public SysIdRoutine getPivotSysIdRoutine() {
        return SysId.getRoutine(
            1, 
            5, 
            "Froggy Pivot", 
            voltage -> setPivotVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngle().getRotations(), 
            () -> pivotMotor.getVelocity().getValueAsDouble(), 
            () -> pivotMotor.getMotorVoltage().getValueAsDouble(), 
            getInstance()
        );
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(pivotMotor.getPosition().getValueAsDouble());
    }

    private Rotation2d getCurrentAngleFromAbsoluteEncoder() {
        double angleRotations = absoluteEncoder.get() - Constants.Froggy.ANGLE_OFFSET.getRotations();
        return Rotation2d.fromRotations(angleRotations > Constants.Froggy.MAXIMUM_ANGLE.getRotations() + Units.degreesToRotations(10)
            ? angleRotations - 1
            : angleRotations);
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(
                getPivotState().getTargetAngle().getDegrees(),
                Constants.Froggy.MINIMUM_ANGLE.getDegrees(),
                Constants.Froggy.MAXIMUM_ANGLE.getDegrees()));
    }

    @Override
    public boolean isAtTargetAngle() {
        return Math.abs(getTargetAngle().getDegrees() - getCurrentAngle().getDegrees()) < Settings.Froggy.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public void setPivotVoltageOverride(Optional<Double> voltage) {
        this.pivotVoltageOverride = voltage;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!hasUsedAbsoluteEncoderToSetPivot && getCurrentAngleFromAbsoluteEncoder().getRotations() != 0) {
            pivotMotor.setPosition(getCurrentAngleFromAbsoluteEncoder().getRotations());
            hasUsedAbsoluteEncoderToSetPivot = true;
        }

        if (Settings.EnabledSubsystems.FROGGY.get()) {
            rollerMotor.set(getRollerState().getTargetSpeed());
            if (pivotVoltageOverride.isPresent()) {
                pivotMotor.setVoltage(pivotVoltageOverride.get());
            } 
            else {
                pivotMotor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations()));
            }
        }
        else {
            rollerMotor.set(0);
            pivotMotor.setVoltage(0);
        }

        // PIVOT
        SmartDashboard.putBoolean("Froggy/Pivot/At Target Angle", isAtTargetAngle());

        SmartDashboard.putNumber("Froggy/Pivot/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Pivot/Target Angle (deg)", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/Pivot/Angle Error (deg)", Math.abs(getTargetAngle().getDegrees() - getCurrentAngle().getDegrees()));

        if (Settings.DEBUG_MODE) {
            // PIVOT
            SmartDashboard.putNumber("Froggy/Pivot/Setpoint (deg)", debuggingMotionProfile.get(getTargetAngle().getDegrees()));
            SmartDashboard.putNumber("Froggy/Pivot/Raw Encoder Angle (deg)", Units.rotationsToDegrees(absoluteEncoder.get()));

            SmartDashboard.putNumber("Froggy/Pivot/Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Froggy/Pivot/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Froggy/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());

            // ROLLER
            SmartDashboard.putNumber("Froggy/Roller/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Froggy/Roller/Supply Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Froggy/Roller/Stator Current", rollerMotor.getStatorCurrent().getValueAsDouble());
        }
    }
}
