package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToL3Front extends ArmMoveToAngle{
    public ArmMoveToL3Front(){
        super(Settings.Arm.L3_ANGLE_FRONT);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}