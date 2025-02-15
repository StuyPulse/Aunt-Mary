package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

public class SuperStructureToFeed extends SuperStructureToState{
    public SuperStructureToFeed() {
        super(SuperStructureTargetState.FEED);
    }
}
