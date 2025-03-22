
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.network.SmartBoolean;

public class ShooterSim extends Shooter {

    private SmartBoolean hasCoral;

    protected ShooterSim() {
        super();
        hasCoral = new SmartBoolean("Shooter/Has Coral", false);
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
