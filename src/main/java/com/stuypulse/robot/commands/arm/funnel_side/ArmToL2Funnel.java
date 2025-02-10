/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm.funnel_side;

import com.stuypulse.robot.commands.arm.ArmToAngle;
import com.stuypulse.robot.constants.Settings;

public class ArmToL2Funnel extends ArmToAngle {
    public ArmToL2Funnel() {
        super(Settings.Arm.L2_ANGLE_FUNNEL);
    }

    @Override
    public void initialize() {
        arm.setRotateBoolean(true);
        super.initialize();
    }
}
