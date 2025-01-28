package com.stuypulse.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climb extends SubsystemBase {
    
    public abstract double getDegrees();

    public abstract double getTargetDegrees();

    public abstract void stop();

    public abstract void periodic();

}




