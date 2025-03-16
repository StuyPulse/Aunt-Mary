/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.funnel;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartBoolean;

public class FunnelSim extends Funnel {

    private final SmartBoolean hasCoral;

    protected FunnelSim() {
        super();
        hasCoral = new SmartBoolean("Funnel/Has Coral", false);
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public boolean shouldReverse() {
        return false;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
