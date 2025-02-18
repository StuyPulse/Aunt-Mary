package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmOffsetTargetDown extends InstantCommand{
    private final Arm arm;

    public ArmOffsetTargetDown() {
        this.arm = Arm.getInstance();
    }

    @Override
    public void initialize() {
        arm.setOperatorOffset(arm.getOperatorOffset().minus(Settings.Operator.Arm.ANGLE_OFFSET_PER_CLICK));
    }
}
