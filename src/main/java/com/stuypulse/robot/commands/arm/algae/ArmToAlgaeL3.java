package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToAlgaeL3 extends ArmToAngle {
    public ArmToAlgaeL3() {
        super(Settings.Arm.ALGAE_L3_ANGLE);
    }

    @Override
    public void initialize() {
        arm.setRotateBoolean(false);
        super.initialize();
    }
}
