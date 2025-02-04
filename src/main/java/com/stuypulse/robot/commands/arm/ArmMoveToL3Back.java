/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmMoveToL3Back extends ArmMoveToAngle {
    public ArmMoveToL3Back() {
        super(Settings.Arm.L3_ANGLE_BACK);
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
