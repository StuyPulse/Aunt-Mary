package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralShooterImpl extends CoralShooter {

    private final TalonFX driveMotor;
    private final DigitalInput motorBeam;
    private final BStream hasCoral;
    
    
    public CoralShooterImpl(){
                
        driveMotor = new TalonFX(Ports.Shooter.MOTOR);
        motorBeam = new DigitalInput(Ports.Shooter.BEAM);
        hasCoral = BStream.create(motorBeam).not()
            .filtered(new BDebounce.Both(Settings.Shooter.BB_DEBOUNCE));

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Shooter.FeedForward.kS.getAsDouble();
        slot0.kV = Settings.Shooter.FeedForward.kV.getAsDouble();
        slot0.kA = Settings.Shooter.FeedForward.kA.getAsDouble();
        
        slot0.kP = Settings.Shooter.PID.kP.getAsDouble();
        slot0.kI = Settings.Shooter.PID.kI.getAsDouble();
        slot0.kD = Settings.Shooter.PID.kD.getAsDouble();

        driveConfig.Slot0 = slot0;

        driveMotor.getConfigurator().apply(driveConfig);
    }

    private double getShooterRPM() {
        return Math.abs(driveMotor.get() * Settings.Shooter.MAX_SHOOTER_RPM);
    }

    public void setShooterRPM(double targetRPM) {
        driveMotor.set(targetRPM / Settings.Shooter.MAX_SHOOTER_RPM);
    }
    
    @Override
    public boolean hasCoral() {
        if 
        return hasCoral.get();
       
    }

    // @Override
    // public boolean hasAlgae() {
    //     return false;
    // }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putBoolean("Shooter/Has Coral", hasCoral());

        SmartDashboard.putNumber("Shooter/RPM", getShooterRPM());

        SmartDashboard.putNumber("Shooter/Voltage", driveMotor.get() * driveMotor.getDutyCycle().getValue()); // can someone check whether this actually gets the voltage
    }
}
