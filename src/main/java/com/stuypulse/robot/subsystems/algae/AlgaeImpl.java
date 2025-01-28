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

public class AlgaeImpl extends Algae {

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private CANcoder pivotEncoder;

    private final AngleController pivotController;

    public double targetAngle;
    public double currentAngle;
    
    public AlgaeImpl() {        
        rollerMotor = new TalonFX(Ports.Algae.ROLLER_PORT);
        pivotMotor = new TalonFX(Ports.Algae.PIVOT_PORT);
        pivotEncoder = new CANcoder(Ports.Algae.PIVOT_ENCODER);
        

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Algae.PID.kS;
        slot0.kV = Settings.Algae.PID.kV;
        slot0.kA = Settings.Algae.PID.kA;
        slot0.kP = Settings.Algae.PID.kP;
        slot0.kI = Settings.Algae.PID.kI;
        slot0.kD = Settings.Algae.PID.kD;     

        pivotConfig.Slot0 = slot0;

        

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(0); // not sure if this should be here

        pivotController = new AnglePIDController(Settings.Algae.PID.kP, Settings.Algae.PID.kI, Settings.Algae.PID.kD);
        
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
        return currentAngle;
    }
    
    @Override
    public void intakeAlgae() {
        rollerMotor.set(Settings.Algae.ALGAE_INTAKE_SPEED);  // CHECK WHAT THE VALUE IS
    }

    @Override 
    public void outakeAlgae() {
        rollerMotor.set(Settings.Algae.ALGAE_OUTTAKE_SPEED); //Check what value is :3
    }

    @Override
    public void intakeCoral() {
        rollerMotor.set(Settings.Algae.CORAL_INTAKE_SPEED); // CHECK WHAT THE VALUE IS
    }

    
    public void outakeCoral() {
        rollerMotor.set(Settings.Algae.CORAL_OUTTAKE_SPEED); // check this value :3
    }




    @Override
    public boolean hasAlgae() {
        return true; // for now
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
