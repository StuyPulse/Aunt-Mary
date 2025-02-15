package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToStow extends SuperStructureToState{
    public SuperStructureToStow() {
        super(SuperStructureTargetState.STOW);
    }
}
