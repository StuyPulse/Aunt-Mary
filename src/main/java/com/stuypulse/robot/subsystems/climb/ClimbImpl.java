package com.stuypulse.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbImpl extends Climb {
    private final TalonFX climbMotor;
    private final CANcoder climbEncoder;
    private double targetDegrees;

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
        climbMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        climbMotor.getConfigurator().apply(climbMotorConfig);
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
        climbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        PositionVoltage controlRequest = new PositionVoltage(0).withSlot(0);
        climbMotor.setControl(controlRequest.withPosition(getTargetDegrees()));

        SmartDashboard.putNumber("Climb/Current Angle (deg)", getDegrees());
        SmartDashboard.putNumber("Climb/Target Angle (deg)", getTargetDegrees());

        SmartDashboard.putNumber("Climb/Motor Voltage", climbMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Supply Voltage", climbMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Motor Current", climbMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Supply Current", climbMotor.getSupplyCurrent().getValueAsDouble());
        
    }
}