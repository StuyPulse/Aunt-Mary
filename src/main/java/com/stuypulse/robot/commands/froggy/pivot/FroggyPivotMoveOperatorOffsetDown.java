package com.stuypulse.robot.commands.froggy.pivot;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.froggy.Froggy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FroggyPivotMoveOperatorOffsetDown extends Command{
    private final Froggy froggy;

    public FroggyPivotMoveOperatorOffsetDown() {
        this.froggy = Froggy.getInstance();
    }

    @Override
    public void execute() {
        Rotation2d dA = Settings.Operator.Froggy.MANUAL_ROTATION_VELOCITY.times(Settings.DT).times(-1);
        froggy.setPivotOperatorOffset(froggy.getPivotOperatorOffset().plus(dA));
    }
}
