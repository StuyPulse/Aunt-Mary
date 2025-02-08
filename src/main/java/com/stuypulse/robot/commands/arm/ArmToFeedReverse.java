package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmToFeedReverse extends ArmToAngle{
    public ArmToFeedReverse(){
        super(Settings.Arm.FUNNEL_ANGLE);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(true);
        super.initialize();
    }
}