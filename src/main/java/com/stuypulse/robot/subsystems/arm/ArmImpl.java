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
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ArmImpl extends Arm {

    private TalonFX motor;
    // private CANcoder absoluteEncoder;
    private DutyCycleEncoder absoluteEncoder;

    private Controller controller;

    public ArmImpl() {
        motor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(motor);

        // absoluteEncoder = new CANcoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(true);

        // MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs()
        //     .withMagnetOffset(Constants.Arm.ANGLE_OFFSET)
        //     .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        // absoluteEncoder.getConfigurator().apply(magnetConfig);

        MotionProfile motionProfile = new MotionProfile(Settings.Arm.MAX_VEL_DEG_PER_S, Settings.Arm.MAX_ACCEL_DEG_PER_S_PER_S);
        motionProfile.reset(Settings.Arm.STOW_ANGLE.getDegrees());

        controller = new MotorFeedforward(Gains.Arm.FF.kS, Gains.Arm.FF.kV, Gains.Arm.FF.kA).position()
            .add(new ArmFeedforward(Gains.Arm.FF.kG_EMPTY))
            .add(new PIDController(Gains.Arm.PID.kP, Gains.Arm.PID.kI, Gains.Arm.PID.kD))
            .setSetpointFilter(motionProfile);
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.Arm.ANGLE_TOLERANCE_DEGREES;
    }

    private Rotation2d getTargetAngle() {
       return getState().getTargetAngle();
    }

    @Override
    public Rotation2d getCurrentAngle() {
        // return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
        return Rotation2d.fromRotations(absoluteEncoder.get() - Constants.Arm.ANGLE_OFFSET.getRotations());
    }

    @Override
    public void periodic() {
        super.periodic();
        
        motor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations()));
        motor.setVoltage(controller.update(getTargetAngle().getRotations(), getCurrentAngle().getRotations()));

        SmartDashboard.putNumber("Arm/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Climb/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
