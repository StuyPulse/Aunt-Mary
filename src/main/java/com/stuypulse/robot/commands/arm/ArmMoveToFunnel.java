package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToFunnel extends ArmMoveToAngle{
    public ArmMoveToFunnel(){
        super(Settings.Arm.FUNNEL_ANGLE);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}