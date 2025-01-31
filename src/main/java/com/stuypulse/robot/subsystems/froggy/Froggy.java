package com.stuypulse.robot.subsystems.froggy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Froggy extends SubsystemBase {

    public static final Froggy instance;
    
    static {
        instance = new FroggyImpl();
    }    
    
    public Froggy() {
        
    }

    public static Froggy getInstance(){
        return instance;
    }

    public abstract void setTargetAngle(double targetAngle);

    public abstract double getTargetAngle();
    
    public abstract double getCurrentAngle(); 
    
    public abstract void intakeAlgae();

    public abstract void intakeCoral();
    
    public abstract void outakeAlgae();

    public abstract void outakeCoral();

    public abstract boolean hasCoral();

    public abstract boolean hasAlgae();
    
    public abstract void stopRoller();

    public abstract void stopPivot();
}
