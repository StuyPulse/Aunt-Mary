package com.stuypulse.robot.commands.arm.front_side;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToL3Front extends ArmToAngle{
    public ArmToL3Front(){
        super(Settings.Arm.L3_ANGLE_FRONT);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(false);
        super.initialize();
    }
}