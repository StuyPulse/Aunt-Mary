package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToL4Front extends ArmMoveToAngle{
    public ArmMoveToL4Front(){
        super(Settings.Arm.L4_ANGLE_FRONT);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}