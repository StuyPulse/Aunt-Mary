package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.subsystems.climb.Climb;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ClimbWaitUntilAtTarget extends WaitUntilCommand{
    public ClimbWaitUntilAtTarget() {
        super(() -> Climb.getInstance().atTargetAngle());
    }
}
