package com.stuypulse.robot.commands.arm.funnel_side;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToL2Funnel extends ArmToAngle{
    public ArmToL2Funnel(){
        super(Settings.Arm.L2_ANGLE_BACK);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(true);
        super.initialize();
    }
}
