package com.stuypulse.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralFunnel extends SubsystemBase{
    
    private static final CoralFunnel instance;

    static {
        instance = new CoralFunnel();
    }

    public CoralFunnel getInstance() {
        return instance;
    }

}
