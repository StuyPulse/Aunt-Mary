/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.DutyCycleOut; 
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbImpl extends Climb {
    private final TalonFX motor;

    private final DutyCycleEncoder absoluteEncoder;

    private final Controller controller;

    private Rotation2d targetAngle;

    private boolean isClimbing;

    public ClimbImpl() {
        motor = new TalonFX(Ports.Climb.MOTOR);
        Motors.Climb.MOTOR_CONFIG.configure(motor);

        controller = new MotorFeedforward(Settings.Climb.FF.kS, Settings.Climb.FF.kV, Settings.Climb.FF.kA).position()
            .add(new ArmFeedforward(Settings.Climb.FF.kG))
            .add(new PIDController(Settings.Climb.PID.kP, Settings.Climb.PID.kI, Settings.Climb.PID.kD))
            .setOutputFilter(x -> SLMath.clamp(x, Settings.Climb.MIN_VOLTAGE, Settings.Climb.MAX_VOLTAGE));

        absoluteEncoder = new DutyCycleEncoder(Ports.Climb.ABSOLUTE_ENCODER, 1.0, Constants.Climb.ANGLE_OFFSET.getRotations());

        // MagnetSensorConfigs magnetSensorConfigs =
        //         new MagnetSensorConfigs()
        //                 .withMagnetOffset(Constants.Climb.ANGLE_OFFSET.getRotations());

        // absoluteEncoder.getConfigurator().apply(magnetSensorConfigs);
        
        absoluteEncoder.setInverted(false);

        targetAngle = new Rotation2d();

        isClimbing = false;
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        isClimbing = false;
        this.targetAngle = targetAngle;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.get());
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void setDutyCycle(double speed){
        motor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getAngle().getDegrees() - targetAngle.getDegrees())
                < Settings.Climb.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public void climb() {
        this.isClimbing = true;
    }

    @Override
    public void periodic() {
        if (isClimbing) {
            motor.setVoltage(Settings.Climb.CLIMB_VOLTAGE);
        } else if (atTargetAngle()) {
            motor.stopMotor();
        } else {
            controller.update(getTargetAngle().getRotations(), getAngle().getRotations());
            motor.setVoltage(controller.getOutput());
        }

        SmartDashboard.putNumber("Climb/Current Angle (deg)", getAngle().getDegrees());
        SmartDashboard.putNumber("Climb/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Climb/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Motor Current", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Climb/Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    }
}
