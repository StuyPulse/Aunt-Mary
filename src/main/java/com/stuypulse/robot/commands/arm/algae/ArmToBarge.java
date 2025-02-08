package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToBarge extends ArmToAngle {
    public ArmToBarge() {
        super(Settings.Arm.BARGE_ANGLE);
    }

    @Override
    public void initialize() {
        arm.setRotateBoolean(true);
        super.initialize();
    }
}
