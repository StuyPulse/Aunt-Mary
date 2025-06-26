/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.superStructure.algae;

import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

public class SuperStructureGolfTeeAlgaePickup extends SuperStructureSetState {
    public SuperStructureGolfTeeAlgaePickup() {
        super(SuperStructureState.GOLF_TEE_ALGAE_PICKUP);
    }
}
