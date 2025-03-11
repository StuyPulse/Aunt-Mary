package com.stuypulse.robot.commands.superStructure;

import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructureSetState extends InstantCommand{
    public SuperStructureSetState(SuperStructureState state) {
        super(() -> SuperStructure.getInstance().setState(state));
    }
}
