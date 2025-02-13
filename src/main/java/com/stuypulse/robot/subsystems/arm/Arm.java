/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.arm.ArmVisualizer;
import com.stuypulse.robot.subsystems.elevator.ElevatorImpl;
import com.stuypulse.robot.subsystems.elevator.ElevatorSimu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    public static final Arm instance;
    public final ArmVisualizer visualizer;

    static {
        if (Robot.isReal()) {
            instance = new ArmImpl();
        } else {
            instance = new ArmSim();
        }
    }

    public static Arm getInstance() {
        return instance;
    }

    public Arm() {
        visualizer = ArmVisualizer.getInstance();
    }

    public abstract Rotation2d getTargetAngle();

    public abstract void setTargetAngle(Rotation2d TargetAngle);

    public abstract boolean atTargetAngle();

    public abstract Rotation2d getCurrentAngle();
        
    public abstract boolean getRotateBoolean();

    public abstract void setRotateBoolean(boolean overElevator);
}
