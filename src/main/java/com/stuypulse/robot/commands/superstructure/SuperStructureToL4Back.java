package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToL4Back extends SuperStructureToState{
    public SuperStructureToL4Back() {
        super(SuperStructureTargetState.L4_BACK);
    }
}
