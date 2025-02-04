/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class CoralFunnel extends SubsystemBase {

    private static final CoralFunnel instance;

    static {
        instance = new CoralFunnelImpl();
    }

    public static CoralFunnel getInstance() {
        return instance;
    }

    public abstract void forward();

    public abstract void reverse();

    public abstract boolean isStalling();

    public abstract boolean hasCoral();

    public abstract void stop();
}
