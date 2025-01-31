 package com.stuypulse.robot.subsystems.froggy;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FroggyImpl extends Froggy {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private final IStream pivotCurrent;
    private final IStream rollerCurrent;

    public double targetAngle;    
    
    public FroggyImpl() {        
        rollerMotor = new TalonFX(Ports.Froggy.ROLLER_PORT);
        pivotMotor = new TalonFX(Ports.Froggy.PIVOT_PORT);
        pivotEncoder = new CANcoder(Ports.Froggy.PIVOT_ENCODER);
        

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Froggy.FF.kS;
        slot0.kV = Settings.Froggy.FF.kV;
        slot0.kA = Settings.Froggy.FF.kA;
        slot0.kP = Settings.Froggy.PID.kP;
        slot0.kI = Settings.Froggy.PID.kI;
        slot0.kD = Settings.Froggy.PID.kD;     

        MotionMagicConfigs motionMagicConfigs = pivotConfig.MotionMagic;
        
        motionMagicConfigs.MotionMagicCruiseVelocity = Settings.Froggy.MotionMagic.MAX_VELOCITY; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = Settings.Froggy.MotionMagic.MAX_ACCELERATION; // Target acceleration of 160 rps/s (0.5 seconds)


        pivotConfig.Slot0 = slot0;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Feedback.SensorToMechanismRatio = Settings.Froggy.GEAR_RATIO;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Settings.Froggy.STATOR_CURRENT_LIMIT; 
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Froggy.PIVOT_CURRENT_LIMIT; 
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Froggy.PIVOT_CURRENT_LIMIT;
        
        pivotConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Settings.Froggy.ROLLER_FF_RAMPING;
        pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Settings.Froggy.ROLLER_PID_RAMPING;
        
        pivotConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Froggy.PIVOT_PID_RAMPING;
        pivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Froggy.PIVOT_FF_RAMPING;
        
        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.getConfigurator().apply(motionMagicConfigs);
        pivotMotor.setPosition(0); // not sure if this should be here

        pivotCurrent = IStream.create(() -> pivotMotor.getStatorCurrent().getValueAsDouble()) // no idea if these work
            .filtered(new HighPassFilter(Settings.Froggy.PIVOT_CURRENT_THRESHOLD));

        rollerCurrent = IStream.create(() -> rollerMotor.getStatorCurrent().getValueAsDouble()) 
            .filtered(new HighPassFilter(Settings.Froggy.ROLLER_CURRENT_THRESHOLD));

    }
    
    @Override
    public double getTargetAngle() {
        return targetAngle + Settings.Froggy.ANGLE_OFFSET;
    }
    
    @Override
    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }
    
    @Override
    public double getCurrentAngle() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360; // i think its ok idk
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
        return pivotCurrent.get() > Settings.Froggy.PIVOT_CURRENT_THRESHOLD;
    }

    @Override
    public boolean hasCoral() {
        return rollerCurrent.get() > Settings.Froggy.ROLLER_CURRENT_THRESHOLD;
    }
    @Override
    public void stopRoller(){
        rollerMotor.set(0);
    }
    @Override
    public void stopPivot(){
        pivotMotor.set(0);
    }

    @Override
    public void periodic() {
        final PositionVoltage controllerOutput = new PositionVoltage(targetAngle);

        pivotMotor.setControl(controllerOutput);

        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(Settings.Froggy.MotionMagic.MAX_VELOCITY);
        rollerMotor.setControl(m_request);
    
        SmartDashboard.putNumber("Froggy/angle", getCurrentAngle());
        SmartDashboard.putNumber("Froggy/pivotCurrent", pivotCurrent.get());
        SmartDashboard.putNumber("Froggy/rollerCurrent", rollerCurrent.get());
        SmartDashboard.putNumber("Froggy/targetAngle", targetAngle);

    }
}
