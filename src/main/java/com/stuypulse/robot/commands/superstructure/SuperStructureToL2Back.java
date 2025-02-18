package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToL2Back extends SuperStructureToState{
    public SuperStructureToL2Back() {
        super(SuperStructureTargetState.L2_BACK);
    }
}
