package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToFeed extends ArmToAngle{
    public ArmToFeed(){
        super(Settings.Arm.FUNNEL_ANGLE);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(false);
        super.initialize();
    }
}