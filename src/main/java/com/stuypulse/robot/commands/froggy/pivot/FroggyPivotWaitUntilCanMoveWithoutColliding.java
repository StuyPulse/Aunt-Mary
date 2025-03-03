package com.stuypulse.robot.commands.froggy.pivot;

import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.util.Clearances;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FroggyPivotWaitUntilCanMoveWithoutColliding extends WaitUntilCommand{
    public FroggyPivotWaitUntilCanMoveWithoutColliding(PivotState targetState) {
        super(() -> Clearances.canMoveFroggyWithoutColliding(targetState));
    }
}
