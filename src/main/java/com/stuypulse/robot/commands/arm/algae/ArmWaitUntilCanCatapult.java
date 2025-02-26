package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ArmWaitUntilCanCatapult extends WaitUntilCommand{
    public ArmWaitUntilCanCatapult() {
        super(() -> Arm.getInstance().getCurrentAngle().getDegrees() > Settings.Arm.CATAPULT_SHOOT_ANGLE.getDegrees());
    }
}
