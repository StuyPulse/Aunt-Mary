package com.stuypulse.robot.commands.superStructure.algae;

import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

public class SuperStructureCatapultReady extends SuperStructureSetState{
    public SuperStructureCatapultReady() {
        super(SuperStructureState.CATAPULT_READY);
    }
}
