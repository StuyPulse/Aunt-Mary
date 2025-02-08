package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToAlgaeL2 extends ArmToAngle {
    public ArmToAlgaeL2() {
        super(Settings.Arm.ALGAE_L2_ANGLE);
    }

    @Override
    public void initialize() {
        arm.setRotateBoolean(false);
        super.initialize();
    }
}
