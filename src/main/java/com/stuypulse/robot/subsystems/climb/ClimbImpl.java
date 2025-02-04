/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbImpl extends Climb {
    private final TalonFX climbMotor;
    private final CANcoder climbEncoder;
    private Rotation2d targetAngle;

    public ClimbImpl() {
        climbMotor = new TalonFX(Ports.Climb.CLIMB_MOTOR);
        climbEncoder = new CANcoder(Ports.Climb.CLIMB_ENCODER);

        TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();

        // gains setting
        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Climb.kS;
        slot0.kV = Settings.Climb.kV;
        slot0.kA = Settings.Climb.kA;

        slot0.kP = Settings.Climb.kP;
        slot0.kI = Settings.Climb.kI;
        slot0.kD = Settings.Climb.kD;

        climbMotorConfig.Slot0 = slot0;

        // basic configs
        climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbMotorConfig.Feedback.SensorToMechanismRatio = Settings.Climb.GEAR_RATIO;

        // current limiting
        climbMotorConfig.CurrentLimits.StatorCurrentLimit = Settings.Climb.CURRENT_LIMIT;
        climbMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // ramp motor voltage
        climbMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Climb.RAMP_RATE;
        climbMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Climb.RAMP_RATE;

        // integrate CANcoder readings into controller
        climbMotorConfig.Feedback.FeedbackRemoteSensorID = climbEncoder.getDeviceID();
        climbMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        climbMotor.getConfigurator().apply(climbMotorConfig);
    }

    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void setTargetDegrees(double targetDegrees) {
        this.targetAngle = new Rotation2d(targetDegrees / 360);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public Rotation2d getAngle() {
        return new Rotation2d((climbEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    @Override
    public void stop() {
        climbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        PositionVoltage controlRequest = new PositionVoltage(0).withSlot(0);
        climbMotor.setControl(controlRequest.withPosition(getTargetAngle().getDegrees()));

        SmartDashboard.putNumber("Climb/Current Angle (deg)", getAngle().getDegrees());
        SmartDashboard.putNumber("Climb/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putNumber(
                "Climb/Motor Voltage", climbMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(
                "Climb/Supply Voltage", climbMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber(
                "Climb/Motor Current", climbMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(
                "Climb/Supply Current", climbMotor.getSupplyCurrent().getValueAsDouble());
    }
}
