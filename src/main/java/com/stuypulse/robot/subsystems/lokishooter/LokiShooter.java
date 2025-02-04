/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.lokishooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LokiShooter extends SubsystemBase {

    private static final LokiShooter instance;

    static {
        instance = new LokiShooterImpl();
    }

    public static LokiShooter getInstance() {
        return instance;
    }

    public abstract void setSpeed(double speed);

    public abstract double getSpeed();

    public abstract boolean hasCoral();

    public abstract boolean hasAlgae();
}
