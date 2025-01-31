package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class CoralShooter extends SubsystemBase {
    
    private static final CoralShooter instance;

    static {
        instance = new CoralShooterImpl();
    }

    public static CoralShooter getInstance() {
        return instance;
    }
    
    private final SmartNumber targetRPM;

    public CoralShooter() {
        targetRPM = new SmartNumber("Shooter/Target RPM",  Settings.Shooter.TARGET_SHOOTER_RPM);
    }

    public double getTargetRPM() {
        return targetRPM.get();
    }

    public abstract double getShooterRPM();

    public abstract void setShooterRPM(double targetRPM);

    public abstract boolean hasCoral();

    public abstract boolean hasAlgae();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Target RPM", getTargetRPM());
    }
}

