package com.stuypulse.robot.commands.superStructure;

import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructureSetState extends InstantCommand{
    private final SuperStructure superStructure;
    private final SuperStructureState state;

    public SuperStructureSetState(SuperStructureState state) {
        this.superStructure = SuperStructure.getInstance();
        this.state = state;
    }

    @Override
    public void initialize() {
        superStructure.setState(state);
    }
}
