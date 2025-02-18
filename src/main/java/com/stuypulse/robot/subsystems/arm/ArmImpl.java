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
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;

public class ArmImpl extends Arm {

    private TalonFX motor;
    // private CANcoder absoluteEncoder;
    private DutyCycleEncoder absoluteEncoder;

    private Controller controller;

    private Optional<Double> voltageOverride;
    private Rotation2d operatorOffset;

    private SysIdRoutine sysidRoutine;
    private boolean isRunningSysid;

    public ArmImpl() {
        super();
        motor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Constants.Arm.MIN_ANGLE.getRotations());

        // absoluteEncoder = new CANcoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(true);

        // MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
        //     .withMagnetOffset(Constants.Arm.ANGLE_OFFSET)
        //     .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        // absoluteEncoder.getConfigurator().apply(magnetConfig);

        MotionProfile motionProfile = new MotionProfile(Settings.Arm.MAX_VEL.getRotations(), Settings.Arm.MAX_ACCEL.getRotations());
        motionProfile.reset(Settings.Arm.STOW_ANGLE.getRotations());

        controller = new MotorFeedforward(Gains.Arm.FF.kS, Gains.Arm.FF.kV, Gains.Arm.FF.kA).position()
            .add(new ArmFeedforward(Gains.Arm.FF.kG))
            .add(new PIDController(Gains.Arm.PID.kP, Gains.Arm.PID.kI, Gains.Arm.PID.kD))
            .setSetpointFilter(motionProfile);

        voltageOverride = Optional.empty();
        operatorOffset = Rotation2d.kZero;

        isRunningSysid = false;

        sysidRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, 
                edu.wpi.first.units.Units.Volts.of(5), 
                null,
                state -> SignalLogger.writeString("SysIdArm_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> {
                    motor.setVoltage(output.in(edu.wpi.first.units.Units.Volts));
                    isRunningSysid = true;
                }, 
                state -> {
                    SignalLogger.writeDouble("Arm Position (rotations)", getCurrentAngle().getRotations());
                    SignalLogger.writeDouble("Arm Velocity (rotations per s)", motor.getVelocity().getValueAsDouble());
                    SignalLogger.writeDouble("Arm Voltage", motor.getMotorVoltage().getValueAsDouble());
                }, 
                this));
    }

    @Override
    public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysidRoutine.quasistatic(direction);
    }

    @Override
    public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysidRoutine.dynamic(direction);
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.Arm.ANGLE_TOLERANCE.getDegrees();
    }

    private Rotation2d getTargetAngle() {
       return getState().getTargetAngle().plus(operatorOffset);
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
        
        if (Settings.EnabledSubsystems.ARM.get() && !isRunningSysid) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            }
            else {
                // motor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations()));
                motor.setVoltage(controller.update(getTargetAngle().getRotations(), getCurrentAngle().getRotations()));
            }
        }
        else {
            motor.setVoltage(0);
        }

        SmartDashboard.putNumber("Climb/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
