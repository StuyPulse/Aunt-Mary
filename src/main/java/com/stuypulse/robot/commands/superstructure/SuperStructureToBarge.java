package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToBarge extends SuperStructureToState{
    public SuperStructureToBarge() {
        super(SuperStructureTargetState.BARGE);
    }
}
