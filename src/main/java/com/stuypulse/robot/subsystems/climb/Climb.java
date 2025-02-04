/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climb extends SubsystemBase {
    private static final Climb instance;

    static {
        instance = new ClimbImpl();
    }

    public static Climb getInstance() {
        return instance;
    }

    public abstract void setTargetAngle(Rotation2d targetAngle);

    public abstract void setTargetDegrees(double targetDegrees);

    public abstract Rotation2d getAngle();

    public abstract Rotation2d getTargetAngle();

    public abstract void stop();

    public abstract void periodic();
}
