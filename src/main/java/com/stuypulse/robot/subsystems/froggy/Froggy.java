/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.froggy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Froggy extends SubsystemBase {

    public static final Froggy instance;

    static {
        instance = new FroggyImpl();
    }

    protected Froggy() {}

    public static Froggy getInstance() {
        return instance;
    }

    public abstract void setTargetAngle(Rotation2d targetAngle);

    public abstract boolean isAtTargetAngle();

    public abstract void intakeAlgae();

    public abstract void outakeAlgae();

    public abstract void intakeCoral();

    public abstract void outakeCoral();

    public abstract void holdAlgae();

    public abstract void stopRoller();

    public abstract boolean hasCoral();

    public abstract boolean hasAlgae();
}
