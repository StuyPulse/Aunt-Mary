/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmMoveToL3Front extends ArmMoveToAngle {
    public ArmMoveToL3Front() {
        super(Settings.Arm.L3_ANGLE_FRONT);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
