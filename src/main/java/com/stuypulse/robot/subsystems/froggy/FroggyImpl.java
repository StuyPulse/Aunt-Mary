package com.stuypulse.robot.subsystems.froggy;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.*;

public class FroggyImpl extends Froggy {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private final IStream rollerCurrent;
    private final BStream hasCoral;
    private final BStream hasAlgae;

    public Rotation2d targetAngle = new Rotation2d();
    
    public FroggyImpl() {        
        rollerMotor = new TalonFX(Ports.Froggy.ROLLER_PORT);
        pivotMotor = new TalonFX(Ports.Froggy.PIVOT_PORT);
        pivotEncoder = new CANcoder(Ports.Froggy.PIVOT_ENCODER);
        

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();      

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Froggy.FF.kS;
        slot0.kV = Settings.Froggy.FF.kV;
        slot0.kA = Settings.Froggy.FF.kA;
        slot0.kP = Settings.Froggy.PID.kP;
        slot0.kI = Settings.Froggy.PID.kI;
        slot0.kD = Settings.Froggy.PID.kD;     
        slot0.kG = Settings.Froggy.PID.kG;

        MotionMagicConfigs motionMagicConfigs = pivotConfig.MotionMagic;
        
        motionMagicConfigs.MotionMagicCruiseVelocity = Settings.Froggy.MotionMagic.MAX_VELOCITY; 
        motionMagicConfigs.MotionMagicAcceleration = Settings.Froggy.MotionMagic.MAX_ACCELERATION; 


        pivotConfig.Slot0 = slot0;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Feedback.SensorToMechanismRatio = Settings.Froggy.GEAR_RATIO;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Settings.Froggy.STATOR_CURRENT_LIMIT; 
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Froggy.PIVOT_CURRENT_LIMIT; 
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Froggy.PIVOT_CURRENT_LIMIT;

        pivotConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Froggy.PIVOT_PID_RAMPING;
        pivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Froggy.PIVOT_FF_RAMPING;
        
        rollerConfig.Slot0 = slot0;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.Feedback.SensorToMechanismRatio = Settings.Froggy.GEAR_RATIO;
        rollerConfig.CurrentLimits.StatorCurrentLimit = Settings.Froggy.STATOR_CURRENT_LIMIT; 
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Froggy.PIVOT_CURRENT_LIMIT; 
        rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Froggy.PIVOT_CURRENT_LIMIT;
        rollerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Settings.Froggy.ROLLER_FF_RAMPING;
        rollerConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Settings.Froggy.ROLLER_PID_RAMPING;


        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.getConfigurator().apply(motionMagicConfigs);

        rollerMotor.getConfigurator().apply(rollerConfig);

        rollerCurrent = IStream.create(() -> Math.abs(rollerMotor.getStatorCurrent().getValueAsDouble()));

        hasCoral = BStream.create(() -> rollerCurrent.getAsDouble() > Settings.Froggy.CORAL_CURRENT_THRESHOLD)
            .filtered(new BDebounce.Rising(Settings.Froggy.ROLLER_DEBOUNCE_TIME));

        hasAlgae = BStream.create(() -> rollerCurrent.getAsDouble() > Settings.Froggy.ALGAE_CURRENT_THRESHOLD)
            .filtered(new BDebounce.Rising(Settings.Froggy.ROLLER_DEBOUNCE_TIME));
    }
    
    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
    
    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = Rotation2d.fromDegrees(SLMath.clamp(getCurrentAngle().getDegrees(), Settings.Froggy.MINIMUM_ANGLE, Settings.Froggy.MAXIMUM_ANGLE));
    }
    
    @Override
    public Rotation2d   getCurrentAngle() {
        return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    @Override
    public void intakeAlgae() {
        rollerMotor.set(Settings.Froggy.ALGAE_INTAKE_SPEED);  
    }

    @Override 
    public void outakeAlgae() {
        rollerMotor.set(Settings.Froggy.ALGAE_OUTTAKE_SPEED); 
    }

    @Override
    public void intakeCoral() {
        rollerMotor.set(Settings.Froggy.CORAL_INTAKE_SPEED); 
    }

    public void outakeCoral() {
        rollerMotor.set(Settings.Froggy.CORAL_OUTTAKE_SPEED);
    }

    @Override
    public boolean hasAlgae() {
        return hasAlgae.get();
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }
    @Override
    public void stopRoller(){
        rollerMotor.set(0);
    }



    @Override
    public void periodic() {
        MotionMagicVoltage controllerOutput = new MotionMagicVoltage(targetAngle.getDegrees());
        rollerMotor.setControl(controllerOutput);
    
        SmartDashboard.putNumber("Froggy/angle", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Froggy/rollerCurrent", rollerCurrent.get());
        SmartDashboard.putNumber("Froggy/targetAngle", targetAngle.getDegrees());
        SmartDashboard.putBoolean("Froggy/hasAlgae", hasAlgae());
        SmartDashboard.putBoolean("Froggy/hasCoral", hasCoral());
    }
}
