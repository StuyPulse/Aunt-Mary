package com.stuypulse.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Algae extends SubsystemBase {

    public static final Algae instance;
    
    static {
        instance = new AlgaeImpl();
    }    
    
    public Algae() {
        
    }

    public static Algae getInstance(){
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
    
}
