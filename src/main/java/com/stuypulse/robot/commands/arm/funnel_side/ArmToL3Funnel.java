package com.stuypulse.robot.commands.arm.funnel_side;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToL3Funnel extends ArmToAngle {
    public ArmToL3Funnel(){
        super(Settings.Arm.L3_ANGLE_BACK);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(true);
        super.initialize();
    }
}
