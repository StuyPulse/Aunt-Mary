package com.stuypulse.robot.subsystems.algae;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeImpl extends Algae {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private final IStream pivotCurrent;
    private final IStream rollerCurrent;

    public double targetAngle;    
    
    public AlgaeImpl() {        
        rollerMotor = new TalonFX(Ports.Algae.ROLLER_PORT);
        pivotMotor = new TalonFX(Ports.Algae.PIVOT_PORT);
        pivotEncoder = new CANcoder(Ports.Algae.PIVOT_ENCODER);
        

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Algae.FF.kS;
        slot0.kV = Settings.Algae.FF.kV;
        slot0.kA = Settings.Algae.FF.kA;
        slot0.kP = Settings.Algae.PID.kP;
        slot0.kI = Settings.Algae.PID.kI;
        slot0.kD = Settings.Algae.PID.kD;     

        pivotConfig.Slot0 = slot0;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.0;
        pivotConfig.Feedback.SensorToMechanismRatio = Settings.Algae.GEAR_RATIO;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Settings.Algae.STATOR_CURRENT_LIMIT; 
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Algae.PIVOT_CURRENT_LIMIT; 
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Algae.PIVOT_CURRENT_LIMIT;


        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(0); // not sure if this should be here

        pivotCurrent = IStream.create(() -> pivotMotor.getStatorCurrent().getValueAsDouble()) // no idea if these work
            .filtered(new HighPassFilter(Settings.Algae.PIVOT_CURRENT_THRESHOLD));

        rollerCurrent = IStream.create(() -> rollerMotor.getStatorCurrent().getValueAsDouble()) 
            .filtered(new HighPassFilter(Settings.Algae.ROLLER_CURRENT_THRESHOLD));

        
    }
    
    @Override
    public double getTargetAngle() {
        return targetAngle + Settings.Algae.ANGLE_OFFSET;
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
        rollerMotor.set(Settings.Algae.ALGAE_INTAKE_SPEED);  
    }

    @Override 
    public void outakeAlgae() {
        rollerMotor.set(Settings.Algae.ALGAE_OUTTAKE_SPEED); 
    }

    @Override
    public void intakeCoral() {
        rollerMotor.set(Settings.Algae.CORAL_INTAKE_SPEED); 
    }

    public void outakeCoral() {
        rollerMotor.set(Settings.Algae.CORAL_OUTTAKE_SPEED);
    }

    @Override
    public boolean hasAlgae() {
        return pivotCurrent.get() > Settings.Algae.PIVOT_CURRENT_THRESHOLD;
    }

    @Override
    public boolean hasCoral() {
        return rollerCurrent.get() > Settings.Algae.ROLLER_CURRENT_THRESHOLD;
    }


    @Override
    public void periodic() {
        PositionVoltage controllerOutput = new PositionVoltage(targetAngle);

        pivotMotor.setControl(controllerOutput);

        SmartDashboard.putNumber("Algae/angle", getCurrentAngle());
        SmartDashboard.putNumber("Algae/pivotCurrent", pivotCurrent.get());
        SmartDashboard.putNumber("Algae/rollerCurrent", rollerCurrent.get());
        SmartDashboard.putNumber("Algae/targetAngle", targetAngle);

    }
}
