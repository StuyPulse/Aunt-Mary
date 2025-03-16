package com.stuypulse.robot.commands.superStructure.algae;

import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

public class SuperStructureProcessor extends SuperStructureSetState{
    public SuperStructureProcessor() {
        super(SuperStructureState.PROCESSOR);
    }
}
