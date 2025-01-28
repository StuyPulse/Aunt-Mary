package com.stuypulse.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class ClimbImpl extends Climb {
    private final TalonFX climbMotor;
    private final CANcoder climbEncoder;
    private double targetDegrees;

    public ClimbImpl() {
        climbMotor = new TalonFX(Ports.Climb.CLIMB_MOTOR);
        climbEncoder = new CANcoder(Ports.Climb.CLIMB_ENCODER);
        targetDegrees = 0;

        TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Climb.kS;
        slot0.kV = Settings.Climb.kV;
        slot0.kA = Settings.Climb.kA;
        slot0.kP = Settings.Climb.kP;
        slot0.kI = Settings.Climb.kI;
        slot0.kD = Settings.Climb.kD;

        climbMotorConfig.Slot0 = slot0;
        climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climbMotorConfig.CurrentLimits.StatorCurrentLimit = Settings.Climb.CURRENT_LIMIT;
        climbMotorConfig.Feedback.SensorToMechanismRatio = Settings.Climb.GEAR_RATIO;

        climbMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climbMotor.getConfigurator().apply(climbMotorConfig);
        climbMotor.setPosition(Settings.Climb.REST_ANGLE);

        CANcoderConfiguration climbEncoderConfig = new CANcoderConfiguration();
        
    }

    public void setTargetDegrees(double targetDegrees) {
        this.targetDegrees = targetDegrees;
    }

    @Override
    public double getTargetDegrees() {
        return targetDegrees;
    }
    
    @Override
    public double getDegrees() {
        return (climbEncoder.getAbsolutePosition().getValueAsDouble() * 360) * Settings.Climb.GEAR_RATIO;
    }

    @Override
    public void stop() {
        climbMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        PositionVoltage controlRequest = new PositionVoltage(0).withSlot(0);
        climbMotor.setControl(controlRequest.withPosition(getTargetDegrees()));

    }

    
    
}