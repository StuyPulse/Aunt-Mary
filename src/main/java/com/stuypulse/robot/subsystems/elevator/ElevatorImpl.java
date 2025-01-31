package com.stuypulse.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorImpl extends Elevator {
    private final TalonFX motor;
    private final SmartNumber targetHeight;
    private final DigitalInput bumpSwitchTop, bumpSwitchBottom;

    public ElevatorImpl() {
        motor = new TalonFX(Ports.Elevator.MOTOR);
        targetHeight = new SmartNumber("Elevator/Target Height", Constants.Elevator.MIN_HEIGHT_METERS);

        bumpSwitchBottom = new DigitalInput(Ports.Elevator.BOTTOM_SWITCH);
        bumpSwitchTop = new DigitalInput(Ports.Elevator.TOP_SWITCH);
        
        TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Elevator.FF.kS.getAsDouble();
        slot0.kV = Settings.Elevator.FF.kV.getAsDouble();
        slot0.kA = Settings.Elevator.FF.kA.getAsDouble();
        slot0.kG = Settings.Elevator.FF.kG.getAsDouble();
        slot0.kP = Settings.Elevator.PID.kP.getAsDouble();
        slot0.kI = Settings.Elevator.PID.kI.getAsDouble();
        slot0.kD = Settings.Elevator.PID.kD.getAsDouble();

        elevatorMotorConfig.Slot0 = slot0;
        
        // Base motor configs
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorConfig.Feedback.SensorToMechanismRatio = Settings.Elevator.GEAR_RATIO;

        // Current limiting
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = Settings.Elevator.CURRENT_LIMIT;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Ramp motor voltage
        elevatorMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Elevator.RAMP_RATE;
        elevatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Elevator.RAMP_RATE;
    
        var elevatorMotionMagicConfigs = elevatorMotorConfig.MotionMagic;
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = Settings.Elevator.TARGET_CRUISE_VELOCITY;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = Settings.Elevator.TARGET_ACCELERATION;
        elevatorMotionMagicConfigs.MotionMagicJerk = Settings.Elevator.TARGET_JERK;

        motor.getConfigurator().apply(elevatorMotorConfig);        
    }
    
    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(
            SLMath.clamp(
                height, 
                Constants.Elevator.MIN_HEIGHT_METERS, 
                Constants.Elevator.MAX_HEIGHT_METERS
            )
        );
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }

    @Override
    public double getCurrentHeight() {
        return motor.getPosition().getValueAsDouble() * Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR;
    }

    @Override
    public boolean atTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < Settings.Elevator.HEIGHT_TOLERANCE_METERS.get();
    }

    @Override
    public boolean atBottom() {
        return !bumpSwitchBottom.get();
    }

    @Override
    public boolean atTop() {
        return !bumpSwitchTop.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        final MotionMagicVoltage controlRequest = new MotionMagicVoltage(getTargetHeight()/Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR);
        motor.setControl(controlRequest);

        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Climb/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Motor Current", motor.getStatorCurrent().getValueAsDouble());
   
    }
}