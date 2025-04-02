
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.superStructure.arm;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SettableNumber;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;

public class ArmImpl extends Arm {

    private TalonFX motor;
    private DutyCycleEncoder absoluteEncoder;
    private boolean hasUsedAbsoluteEncoderToSetArm;

    private MotionProfile debuggingMotionProfile;

    private SettableNumber velLimitDegreesPerSecond;
    private SettableNumber accelLimitDegreesPerSecondSquared;

    private SettableNumber kG;

    private Optional<Double> voltageOverride;

    public ArmImpl() {
        super();
        motor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Settings.Arm.MIN_ANGLE.getRotations());

        absoluteEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(true);

        hasUsedAbsoluteEncoderToSetArm = false;

        velLimitDegreesPerSecond = new SettableNumber(Settings.Arm.Constraints.MAX_VEL_TELEOP.getDegrees());
        accelLimitDegreesPerSecondSquared = new SettableNumber(Settings.Arm.Constraints.MAX_VEL_TELEOP.getDegrees());

        kG = new SettableNumber(Gains.Arm.Empty.FF.kG);

        debuggingMotionProfile = new MotionProfile(velLimitDegreesPerSecond, accelLimitDegreesPerSecondSquared);
        debuggingMotionProfile.reset(Settings.Arm.MIN_ANGLE.getDegrees());

        voltageOverride = Optional.empty();
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            2, 
            6, 
            "Arm", 
            voltage -> setVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngle().getRotations(), 
            () -> motor.getVelocity().getValueAsDouble(), 
            () -> motor.getMotorVoltage().getValueAsDouble(), 
            getInstance()
        );
    }

    private boolean isWithinTolerance(Rotation2d tolerance) {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < tolerance.getDegrees();
    }

    @Override
    public boolean atTargetAngle() {
        return isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE);
    }

    @Override
    public boolean atCanSkipClearanceAngle() {
        return isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE_TO_SKIP_CLEARANCE);
    }

    private Rotation2d getTargetAngle() {
       return Rotation2d.fromDegrees(
        SLMath.clamp(getState().getTargetAngle().getDegrees(), Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
    }

    private Rotation2d getCurrentAngleFromAbsoluteEncoder() {
        double encoderAngle = absoluteEncoder.get() - Constants.Arm.ANGLE_OFFSET.getRotations();
        return Rotation2d.fromRotations(encoderAngle > Settings.Arm.MIN_ANGLE.minus(Rotation2d.fromDegrees(15)).getRotations() 
            ? encoderAngle 
            : encoderAngle + 1);
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public double getVoltageOverride() {
        if (voltageOverride.isPresent()) {
            return voltageOverride.get();
        }
        else {
            return 0;
        }
    }

    @Override
    public void setMotionProfileConstraints(Rotation2d velLimit, Rotation2d accelLimit) {
        this.velLimitDegreesPerSecond.set(velLimit.getDegrees());
        this.accelLimitDegreesPerSecondSquared.set(accelLimit.getDegrees());
        Motors.Arm.MOTOR_CONFIG.withMotionProfile(velLimit.getRotations(), accelLimit.getRotations());
        Motors.Arm.MOTOR_CONFIG.configure(motor);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!hasUsedAbsoluteEncoderToSetArm && getCurrentAngleFromAbsoluteEncoder().getRotations() != 0) {
            motor.setPosition(getCurrentAngleFromAbsoluteEncoder().getRotations());
            hasUsedAbsoluteEncoderToSetArm = true;
        }
                
        if (Settings.EnabledSubsystems.ARM.get()) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            }
            else {
                if (getState() == ArmState.FEED 
                    && atTargetAngle()
                    && Shooter.getInstance().getState() == ShooterState.ACQUIRE_CORAL
                    && Elevator.getInstance().getState() == ElevatorState.FEED
                    && Elevator.getInstance().atTargetHeight())
                {
                    motor.setVoltage(-0.9);
                }
                else {
                    if (Shooter.getInstance().hasCoral()) {
                        this.kG.set(Gains.Arm.Coral.FF.kG);
                        motor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations())
                            .withSlot(0));
                    }
                    if (Shooter.getInstance().getState() == ShooterState.HOLD_ALGAE) {
                        this.kG.set(Gains.Arm.Algae.FF.kG);
                        motor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations())
                            .withSlot(1));
                    }
                    else {
                        this.kG.set(Gains.Arm.Empty.FF.kG);
                        motor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations())
                            .withSlot(2));
                    }
                }
            }
        }
        else {
            motor.setVoltage(0);
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Arm/Constraints/Current Max vel (deg per s)", velLimitDegreesPerSecond.get());
            SmartDashboard.putNumber("Arm/Constraints/Current Max accel (deg per s per s)", accelLimitDegreesPerSecondSquared.get());

            SmartDashboard.putNumber("Arm/Current Setpoint (deg)", debuggingMotionProfile.get(getTargetAngle().getDegrees()));

            SmartDashboard.putBoolean("Arm/Is Voltage Override Present", voltageOverride.isPresent());
            SmartDashboard.putNumber("Arm/Voltage Override", getVoltageOverride());

            SmartDashboard.putNumber("Arm/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Arm/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Arm/Stator Current", motor.getStatorCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Arm/Raw Encoder Value (deg)", Units.rotationsToDegrees(absoluteEncoder.get()));
        }
    }
}
