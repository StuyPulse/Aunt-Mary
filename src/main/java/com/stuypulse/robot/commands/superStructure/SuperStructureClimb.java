package com.stuypulse.robot.commands.superStructure;

import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

public class SuperStructureClimb extends SuperStructureSetState{
    public SuperStructureClimb() {
        super(SuperStructureState.CLIMB);
    }
}
