package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmOffsetTargetUp extends InstantCommand{
    private final Arm arm;

    public ArmOffsetTargetUp() {
        this.arm = Arm.getInstance();
    }

    @Override
    public void initialize() {
        arm.setOperatorOffset(arm.getOperatorOffset().plus(Settings.Operator.Arm.ANGLE_OFFSET_PER_CLICK));
    }
}
