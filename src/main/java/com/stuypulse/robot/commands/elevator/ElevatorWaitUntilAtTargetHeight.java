package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ElevatorWaitUntilAtTargetHeight extends WaitUntilCommand{
    public ElevatorWaitUntilAtTargetHeight() {
        super(() -> Elevator.getInstance().atTargetHeight());
    }
}
