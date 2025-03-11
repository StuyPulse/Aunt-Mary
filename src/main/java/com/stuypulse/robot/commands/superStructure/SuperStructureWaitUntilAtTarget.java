package com.stuypulse.robot.commands.superStructure;

import com.stuypulse.robot.subsystems.superStructure.SuperStructure;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SuperStructureWaitUntilAtTarget extends WaitUntilCommand{
    public SuperStructureWaitUntilAtTarget() {
        super(() -> SuperStructure.getInstance().atTarget());
    }
}
