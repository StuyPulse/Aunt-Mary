package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToStow extends ArmToAngle {
    public ArmToStow() {
        super(Settings.Arm.STOW_ANGLE);
    }

    @Override
    public void initialize() {
        arm.setRotateBoolean(false);
        super.initialize();
    }
}
