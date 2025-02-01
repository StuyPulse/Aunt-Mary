package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToL4 extends ArmMoveToAngle{
    public ArmMoveToL4(){
        super(Settings.Arm.L4_ANGLE);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}