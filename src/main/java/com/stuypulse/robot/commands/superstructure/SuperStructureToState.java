package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.SuperStructure;
import com.stuypulse.robot.subsystems.superstructure.SuperStructure.SuperStructureTargetState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructureToState extends InstantCommand{
    private final SuperStructure superStructure;
    private final SuperStructureTargetState targetState;

    public SuperStructureToState(SuperStructureTargetState state) {
        this.superStructure = SuperStructure.getInstance();
        this.targetState = state;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setTargetState(targetState);
    }
}
