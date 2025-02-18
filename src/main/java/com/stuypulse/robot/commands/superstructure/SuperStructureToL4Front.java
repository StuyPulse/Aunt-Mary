package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToL4Front extends SuperStructureToState{
    public SuperStructureToL4Front() {
        super(SuperStructureTargetState.L4_FRONT);
    }
}
