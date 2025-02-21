/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ArmDriveFeedForward;
import com.stuypulse.robot.util.ArmElevatorFeedForward;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmImpl extends Arm {

    private TalonFX motor;
    private DutyCycleEncoder absoluteEncoder;

    private Controller controller;

    private Optional<Double> voltageOverride;
    private Rotation2d operatorOffset;
    private Number kP, kI, kD, kS, kV, kA, kG;

    public ArmImpl() {
        super();
        motor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Settings.Arm.MIN_ANGLE.getRotations());
        
        kG = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.Algae.FF.kG;
            } else if (Shooter.getInstance().hasCoral()) {
                return Gains.Arm.Coral.FF.kG;
            } else {
                return Gains.Arm.Empty.FF.kG;
            }
        }).number();
              
        kS = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.Algae.FF.kS;
            } else if (Shooter.getInstance().hasCoral()) {
                return Gains.Arm.Coral.FF.kS;
            } else {
                return Gains.Arm.Empty.FF.kS;
            }
        }).number();

        kV = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.Algae.FF.kV;
            } else if (Shooter.getInstance().hasCoral()) {
                return Gains.Arm.Coral.FF.kV;
            } else {
                return Gains.Arm.Empty.FF.kV;
            }
        }).number();

        kA = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.Algae.FF.kA;
            } else if (Shooter.getInstance().hasCoral()) {
                return Gains.Arm.Coral.FF.kA;
            } else {
                return Gains.Arm.Empty.FF.kA;
            }
        }).number();

        kP = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.Algae.PID.kP;
            } else if (Shooter.getInstance().hasCoral()) {
                return Gains.Arm.Coral.PID.kP;
            } else {
                return Gains.Arm.Empty.PID.kP;
            }
        }).number();

        kI = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.Algae.PID.kI;
            } else if (Shooter.getInstance().hasCoral()) {
                return Gains.Arm.Coral.PID.kI;
            } else {
                return Gains.Arm.Empty.PID.kI;
            }
        }).number();

        kD = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.Algae.PID.kD;
            } else if (Shooter.getInstance().hasCoral()) {
                return Gains.Arm.Coral.PID.kD;
            } else {
                return Gains.Arm.Empty.PID.kD;
            }
        }).number();

        absoluteEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(true);

        MotionProfile motionProfile = new MotionProfile(Settings.Arm.MAX_VEL.getDegrees(), Settings.Arm.MAX_ACCEL.getDegrees());
        motionProfile.reset(Settings.Arm.MIN_ANGLE.getDegrees());

        controller = new MotorFeedforward(kS, kV, kA).position()
            .add(new ArmFeedforward(kG))
            .add(new ArmDriveFeedForward(kG, CommandSwerveDrivetrain.getInstance()::getXAccelGs))
            .add(new ArmElevatorFeedForward(kG, Elevator.getInstance()::getAccelGs))
            .add(new PIDController(kP, kI, kD))
            .setSetpointFilter(motionProfile);

        voltageOverride = Optional.empty();
        operatorOffset = Rotation2d.kZero;
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            3, 
            7, 
            "Arm", 
            voltage -> setVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngle().getDegrees(), 
            () -> Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble()), 
            () -> motor.getMotorVoltage().getValueAsDouble(), 
            getInstance()
        );
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.Arm.ANGLE_TOLERANCE.getDegrees();
    }

    private Rotation2d getTargetAngle() {
       return Rotation2d.fromDegrees(
        SLMath.clamp(getState().getTargetAngle().plus(operatorOffset).getDegrees(), Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
    }

    @Override
    public Rotation2d getCurrentAngle() {
        double encoderAngle = absoluteEncoder.get();
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
    public void setOperatorOffset(Rotation2d offset) {
        this.operatorOffset = offset;
    }

    @Override
    public Rotation2d getOperatorOffset() {
        return this.operatorOffset;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        if (Settings.EnabledSubsystems.ARM.get()) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            }
            else {
                motor.setVoltage(controller.update(getTargetAngle().getDegrees(), getCurrentAngle().getDegrees()));
            }
        }
        else {
            motor.setVoltage(0);
        }

        SmartDashboard.putBoolean("Arm/Is Voltage Override Present", voltageOverride.isPresent());
        SmartDashboard.putNumber("Arm/Voltage Override", getVoltageOverride());

        SmartDashboard.putNumber("Arm/Setpoint (deg)", controller.getSetpoint());
        SmartDashboard.putNumber("Arm/Angle Error (deg)", controller.getError());
        
        SmartDashboard.putBoolean("Arm/Absolute Encoder is Connected", absoluteEncoder.isConnected());
        SmartDashboard.putNumber("Arm/Absolute Encoder Value raw (deg)", Units.rotationsToDegrees(absoluteEncoder.get()));

        SmartDashboard.putNumber("Arm/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
