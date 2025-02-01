package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToL2 extends ArmMoveToAngle{
    public ArmMoveToL2(){
        super(Settings.Arm.L2_ANGLE);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}