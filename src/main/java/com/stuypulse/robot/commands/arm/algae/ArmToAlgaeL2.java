/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.commands.arm.ArmToAngle;
import com.stuypulse.robot.constants.Settings;

public class ArmToAlgaeL2 extends ArmToAngle {
    public ArmToAlgaeL2() {
        super(Settings.Arm.ALGAE_L2_ANGLE);
    }

    @Override
    public void initialize() {
        arm.setRotateBoolean(false);
        super.initialize();
    }
}
