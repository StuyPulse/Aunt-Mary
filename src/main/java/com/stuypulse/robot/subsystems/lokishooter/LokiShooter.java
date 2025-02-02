package com.stuypulse.robot.subsystems.lokishooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LokiShooter extends SubsystemBase {
    
    private static final LokiShooter instance;

    static {
        instance = new LokiShooterImpl();
    }
    
    public static LokiShooter getInstance() {
        return instance;
    }

    public abstract void setSpeed(double speed);

    public abstract double getSpeed();

    public abstract boolean hasCoral();

    public abstract boolean hasAlgae();
}

