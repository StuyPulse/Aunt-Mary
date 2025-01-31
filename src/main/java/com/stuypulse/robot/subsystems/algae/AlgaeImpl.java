package com.stuypulse.robot.subsystems.algae;
import java.util.Set;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;

public class AlgaeImpl extends Algae {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private final AngleController pivotController;
    private final IStream pivotCurrent;

    public double targetAngle;
    public double currentAngle;

    
    
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

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(0); // not sure if this should be here

        pivotController = new AnglePIDController(Settings.Algae.PID.kP, Settings.Algae.PID.kI, Settings.Algae.PID.kD);

        pivotCurrent = IStream.create(() -> pivotMotor.getStatorCurrent().getValueAsDouble()) // no idea if this works
            .filtered(new HighPassFilter(Settings.Algae.PIVOT_CURRENT_THRESHOLD));
    }
    
    @Override
    public double getTargetAngle() {
        return targetAngle;
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
        return true; // for now
    }

    @Override
    public void periodic() {
        PositionVoltage controllerOutput = new PositionVoltage(targetAngle);
        
        pivotMotor.setControl(controllerOutput);
    }
}
