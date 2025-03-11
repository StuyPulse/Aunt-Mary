package com.stuypulse.robot.commands.superStructure.algae;

import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SuperStructureWaitUntilCanCatapult extends WaitUntilCommand{
    public SuperStructureWaitUntilCanCatapult() {
        super(() -> Arm.getInstance().getCurrentAngle().getDegrees() > Settings.Arm.CATAPULT_SHOOT_ANGLE.getDegrees());
    }
}
