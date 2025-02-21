/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

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
    // private CANcoder absoluteEncoder;
    private DutyCycleEncoder absoluteEncoder;

    private Controller controller;

    private Optional<Double> voltageOverride;
    private Rotation2d operatorOffset;
    private Number kS, kV, kA, kG;

    public ArmImpl() {
        super();
        motor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Settings.Arm.MIN_ANGLE.getRotations());
        kS = 0.0;
        kV = 0.0;
        kA = 0.0;
        kG = 0.0;

        // absoluteEncoder = new CANcoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(true);

        // MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
        //     .withMagnetOffset(Constants.Arm.ANGLE_OFFSET)
        //     .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        // absoluteEncoder.getConfigurator().apply(magnetConfig);

        MotionProfile motionProfile = new MotionProfile(Settings.Arm.MAX_VEL.getDegrees(), Settings.Arm.MAX_ACCEL.getDegrees());
        motionProfile.reset(Settings.Arm.MIN_ANGLE.getDegrees());

        controller = new MotorFeedforward(kS, kV, kA).position()
            .add(new ArmFeedforward(kG))
            .add(new ArmDriveFeedForward(kG, CommandSwerveDrivetrain.getInstance()::getXAccelGs))
            .add(new ArmElevatorFeedForward(kG, Elevator.getInstance()::getAccelGs))
            .add(new PIDController(Gains.Arm.PID.kP, Gains.Arm.PID.kI, Gains.Arm.PID.kD))
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
        // double degrees = Units.rotationsToDegrees(absoluteEncoder.get() - Constants.Arm.ANGLE_OFFSET.getRotations());
        // if (degrees < Constants.Arm.MIN_ANGLE.minus(Rotation2d.fromDegrees(5)).getDegrees()) {
        //     degrees += 360;
        // }
        // return Rotation2d.fromDegrees(degrees);
        return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
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

        // Rotation2d absoluteEncoderAngle = Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Arm.ANGLE_OFFSET.getRotations());

        // absoluteEncoderAngle = absoluteEncoderAngle.getDegrees() <= Settings.Arm.MIN_ANGLE.minus(Rotation2d.fromDegrees(10)).getDegrees()
        //     ? absoluteEncoderAngle.plus(Rotation2d.fromRotations(1))
        //     : absoluteEncoderAngle;

        // motor.setPosition(absoluteEncoderAngle.getRotations(), 0.0);

        // if (absoluteEncoderAngle.getRotations() >= Settings.Arm.MIN_ANGLE.minus(Rotation2d.fromDegrees(5)).getRotations() 
        //     && absoluteEncoderAngle.getRotations() <= Constants.Arm.ENCODER_BREAKPOINT_ANGLE.getRotations()
        //     && absoluteEncoder.isConnected()) 
        // {
        //     motor.setPosition(absoluteEncoderAngle.getRotations(), 0.0);
        // }

        kG = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.FF.ALGAE_kG;
            } else if (!Shooter.getInstance().hasCoral()) {
                return Gains.Arm.FF.EMPTY_kG;
            } else {
                return Gains.Arm.FF.CORAL_kG;
            }
        }).number();
              
        kS = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.FF.ALGAE_kS;
            } else if (!Shooter.getInstance().hasCoral()) {
                return Gains.Arm.FF.EMPTY_kS;
            } else {
                return Gains.Arm.FF.CORAL_kS;
            }
        }).number();

        kV = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.FF.ALGAE_kV;
            } else if (!Shooter.getInstance().hasCoral()) {
                return Gains.Arm.FF.EMPTY_kV;
            } else {
                return Gains.Arm.FF.CORAL_kV;
            }
        }).number();

        kA = IStream.create(() -> {
            if (getState() == Arm.ArmState.HOLD_ALGAE || getState() == Arm.ArmState.BARGE) {
                return Gains.Arm.FF.ALGAE_kA;
            } else if (!Shooter.getInstance().hasCoral()) {
                return Gains.Arm.FF.EMPTY_kA;
            } else {
                return Gains.Arm.FF.CORAL_kA;
            }
        }).number();
        
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
        // SmartDashboard.putNumber("Arm/Absolute Encoder Value (deg)", absoluteEncoderAngle.getDegrees());

        SmartDashboard.putNumber("Arm/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
