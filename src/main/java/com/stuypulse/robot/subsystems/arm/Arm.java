/************************ PROJECT MARY***********/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.subsystems.arm.ArmVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    public static final Arm instance;
    public final ArmVisualizer visualizer;

    static {
        instance = new ArmImpl();
    }

    public static Arm getInstance() {
        return instance;
    }

    public Arm() {
        visualizer = ArmVisualizer.getInstance();
    }

    public abstract void setTargetAngle(Rotation2d TargetAngle);

    public abstract Rotation2d getTargetAngle();

    public abstract Rotation2d getArmAngle();

    public abstract void setRotateBoolean(boolean overElevator);

    public abstract boolean atTargetAngle();
}
