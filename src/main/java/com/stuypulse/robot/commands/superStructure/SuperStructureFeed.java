package com.stuypulse.robot.commands.superStructure;

import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

public class SuperStructureFeed extends SuperStructureSetState{
    public SuperStructureFeed() {
        super(SuperStructureState.FEED);
    }
}
