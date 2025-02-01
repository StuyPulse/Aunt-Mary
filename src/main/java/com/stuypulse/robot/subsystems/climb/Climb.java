package com.stuypulse.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climb extends SubsystemBase {
    private static final Climb instance;

    static {
        instance = new ClimbImpl();
    }
    
    public static Climb getInstance() {
        return instance;
    }

    public abstract void setTargetDegrees(double targetDegrees);

    public abstract double getDegrees();

    public abstract double getTargetDegrees();

    public abstract void stop();

    public abstract void periodic();

}




