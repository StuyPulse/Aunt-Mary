package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToL3Front extends SuperStructureToState{
    public SuperStructureToL3Front() {
        super(SuperStructureTargetState.L3_FRONT);
    }
}
