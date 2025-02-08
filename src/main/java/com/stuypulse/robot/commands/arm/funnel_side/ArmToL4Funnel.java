package com.stuypulse.robot.commands.arm.funnel_side;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToL4Funnel extends ArmToAngle {
    public ArmToL4Funnel(){
        super(Settings.Arm.L4_ANGLE_FUNNEL);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(true);
        super.initialize();
    }
}
