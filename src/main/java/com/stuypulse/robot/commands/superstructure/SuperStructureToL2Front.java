package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToL2Front extends SuperStructureToState{
    public SuperStructureToL2Front() {
        super(SuperStructureTargetState.L2_FRONT);
    }
}
