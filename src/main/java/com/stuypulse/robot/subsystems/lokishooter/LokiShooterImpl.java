package com.stuypulse.robot.subsystems.lokishooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LokiShooterImpl extends LokiShooter {

    private final TalonFX shooterMotor;
    private final TalonFXConfiguration shooterConfig;

    private final DigitalInput motorBeam;
    private final BStream hasCoral;
    private final BStream hasAlgae;
    
    private final IStream shooterCurrent;
    
    public LokiShooterImpl(){
        shooterMotor = new TalonFX(Ports.Shooter.MOTOR);

        motorBeam = new DigitalInput(Ports.Shooter.BEAM);
        
        shooterCurrent = IStream.create(() -> shooterMotor.getStatorCurrent().getValueAsDouble());
        
        hasCoral = BStream.create(motorBeam).not()
            .filtered(new BDebounce.Both(Settings.Shooter.BB_DEBOUNCE));
        hasAlgae = BStream.create(this::detectCurrentSpike)
            .filtered(new BDebounce.Both(Settings.Shooter.ALGAE_DEBOUNCE));
        
        shooterConfig = new TalonFXConfiguration();

        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Shooter.DRIVE_CURRENT_LIMIT;
        shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Shooter.DRIVE_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.StatorCurrentLimit = Settings.Shooter.DRIVE_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shooterMotor.getConfigurator().apply(shooterConfig);
    }

    @Override
    public void setSpeed(double speed){
        shooterMotor.set(speed);
    }

    @Override
    public double getSpeed(){
        return shooterMotor.get();
    }
    
    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public boolean hasAlgae(){
        return hasAlgae.get();
    }

    // Uses stall detection to determine if the shooter has algae by checking if the current is above a certain threshold
    private boolean detectCurrentSpike() {
        return shooterCurrent.get() > Settings.Shooter.DRIVE_CURRENT_THRESHOLD;
    }
    

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Shooter/Voltage", shooterMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Current", shooterCurrent.get());

        SmartDashboard.putNumber("Shooter/Speed", getSpeed());

        SmartDashboard.putBoolean("Shooter/Has Coral", hasCoral());        
        SmartDashboard.putBoolean("Shooter/Has Algae", hasAlgae());
    }
}
