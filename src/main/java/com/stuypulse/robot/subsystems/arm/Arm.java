package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class Arm extends SubsystemBase {

    public static final Arm instance;

    static {
        instance = new ArmImpl();
    }

    public static Arm getInstance() {
        return instance;
    }

    public abstract void setTargetAngle(Rotation2d TargetAngle);

    public abstract Rotation2d getTargetAngle();

    public abstract Rotation2d getArmAngle();

    // public abstract void setVoltage(double voltage);
}