package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SuperStructureWaitUntilAtTarget extends ParallelCommandGroup{
    public SuperStructureWaitUntilAtTarget() {
        super(
            new ArmWaitUntilAtTarget(),
            new ElevatorWaitUntilAtTargetHeight()
        );
    }
}
