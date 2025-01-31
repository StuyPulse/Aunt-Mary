package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToL3 extends ArmMoveToAngle{
    public ArmMoveToL3(){
        super(Settings.Arm.L3_ANGLE);
    }

}