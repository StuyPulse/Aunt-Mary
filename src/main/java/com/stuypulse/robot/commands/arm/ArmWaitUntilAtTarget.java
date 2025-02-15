package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ArmWaitUntilAtTarget extends WaitUntilCommand{
    public ArmWaitUntilAtTarget() {
        super(() -> Arm.getInstance().atTargetAngle());
    }
}
