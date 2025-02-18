package com.stuypulse.robot.commands.froggy.pivot;

import com.stuypulse.robot.subsystems.froggy.Froggy;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FroggyPivotWaitUntilAtTargetAngle extends WaitUntilCommand{
    public FroggyPivotWaitUntilAtTargetAngle() {
        super(() -> Froggy.getInstance().isAtTargetAngle());
    }
}
