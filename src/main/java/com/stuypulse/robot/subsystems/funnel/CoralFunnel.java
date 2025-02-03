package com.stuypulse.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class CoralFunnel extends SubsystemBase{
    
    private static final CoralFunnel instance;

    static {
        instance = new CoralFunnelImpl();
    }

    public static CoralFunnel getInstance() {
        return instance;
    }

    public abstract void forward();

    public abstract void reverse();

    public abstract boolean isStalling();

    public abstract boolean hasCoral();
    
    public abstract void stop();

}
