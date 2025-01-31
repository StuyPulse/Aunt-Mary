package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralShooterImpl extends CoralShooter {

    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveConfig;

    private final DigitalInput motorBeam;
    private final BStream hasCoral;
    private boolean hasAlgae;

    private final IStream driveCurrent;
    
    public CoralShooterImpl(){
        driveMotor = new TalonFX(Ports.Shooter.MOTOR);

        motorBeam = new DigitalInput(Ports.Shooter.BEAM);
        hasCoral = BStream.create(motorBeam).not()
            .filtered(new BDebounce.Both(Settings.Shooter.BB_DEBOUNCE));

        driveConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Shooter.FeedForward.kS.getAsDouble();
        slot0.kV = Settings.Shooter.FeedForward.kV.getAsDouble();
        slot0.kA = Settings.Shooter.FeedForward.kA.getAsDouble();
        
        slot0.kP = Settings.Shooter.PID.kP.getAsDouble();
        slot0.kI = Settings.Shooter.PID.kI.getAsDouble();
        slot0.kD = Settings.Shooter.PID.kD.getAsDouble();

        driveConfig.Slot0 = slot0;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = Settings.Shooter.GEAR_RATIO;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Shooter.DRIVE_CURRENT_LIMIT;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Shooter.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimit = Settings.Shooter.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        driveConfig.TorqueCurrent.TorqueNeutralDeadband = 0.05;
        
        driveCurrent = IStream.create(() -> driveMotor.getStatorCurrent().getValueAsDouble())
            .filtered(new HighPassFilter(Settings.Shooter.DRIVE_CURRENT_THRESHOLD));

        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setPosition(0);
    }

    public double getShooterRPM() {
        return driveMotor.get() * Settings.Shooter.MAX_SHOOTER_RPM;
    }

    public void setShooterRPM(double targetRPM) {
        this.setTargetRPM(targetRPM);
        driveMotor.set(targetRPM / Settings.Shooter.MAX_SHOOTER_RPM);
    }
    
    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public boolean hasAlgae(){
        return this.hasAlgae;
    }

    // Uses stall detection to determine if the shooter has algae by checking if the current is above a certain threshold
    public boolean detectCurrentSpike() {
        return driveCurrent.get() > Settings.Shooter.DRIVE_CURRENT_THRESHOLD;
    }
    

    @Override
    public void periodic() {
        super.periodic();

        if (this.getTargetRPM() != 0){
            this.hasAlgae = this.detectCurrentSpike();
        }

        SmartDashboard.putNumber("Shooter/RPM", getShooterRPM());

        SmartDashboard.putNumber("Shooter/Voltage", driveMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Current", driveCurrent.get());

        SmartDashboard.putBoolean("Shooter/Has Coral", hasCoral());        
        SmartDashboard.putBoolean("Shooter/Has Algae", hasAlgae());
    }
}
