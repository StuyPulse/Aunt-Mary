package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class Arm extends SubsystemBase {

    public static Arm getInstance() {
        return instance;
    }

}