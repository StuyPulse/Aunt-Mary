package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class Arm extends SubsystemBase {

    public static final Arm instance;

    static {
        instance = new ArmImpl();
    }
    public static Arm getInstance() {
        return instance;
    }

    public abstract void setTargetAngle(double TargetAngle);

    public abstract double getTargetAngle();

    public abstract double getArmAngle();

    // public abstract void setVoltage(double voltage);
}