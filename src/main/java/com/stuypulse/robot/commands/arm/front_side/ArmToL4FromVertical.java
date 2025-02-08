package com.stuypulse.robot.commands.arm.front_side;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.arm.ArmToAngle;

public class ArmToL4FromVertical extends ArmToAngle{
    public ArmToL4FromVertical(){
        super(Settings.Arm.L4_ANGLE_FRONT);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(false);
        super.initialize();
    }
}