/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;

public class ArmToAlgaeL2 extends ArmSetState {
    public ArmToAlgaeL2() {
        super(ArmState.ALGAE_L2);
    }
}
