package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToL3Back extends SuperStructureToState{
    public SuperStructureToL3Back() {
        super(SuperStructureTargetState.L3_BACK);
    }
}
