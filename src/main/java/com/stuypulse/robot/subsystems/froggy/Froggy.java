package com.stuypulse.robot.subsystems.froggy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Froggy extends SubsystemBase {

    public static final Froggy instance;
    
    static {
        instance = new FroggyImpl();
    }    
    
    protected Froggy() {
        
    }

    public static Froggy getInstance(){
        return instance;
    }

    public abstract void setTargetAngle(Rotation2d targetAngle);

    public abstract Rotation2d getTargetAngle();
    
    public abstract Rotation2d getCurrentAngle(); 
    
    public abstract void intakeAlgae();

    public abstract void intakeCoral();
    
    public abstract void outakeAlgae();

    public abstract void outakeCoral();

    public abstract boolean hasCoral();

    public abstract boolean hasAlgae();
    
    public abstract void stopRoller();

    public abstract void holdAlgae();
}
