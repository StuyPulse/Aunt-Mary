/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToVertical extends ArmToAngle {
    public ArmToVertical() {
        super(Settings.Arm.VERTICAL_ANGLE);
    }

    @Override
    public void initialize() {
        arm.setRotateBoolean(false);
        super.initialize();
    }
}
