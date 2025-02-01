package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToL2Front extends ArmMoveToAngle{
    public ArmMoveToL2Front(){
        super(Settings.Arm.L2_ANGLE_FRONT);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}