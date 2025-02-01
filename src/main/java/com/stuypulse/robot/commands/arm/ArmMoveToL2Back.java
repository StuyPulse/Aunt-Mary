package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmMoveToL2Back extends ArmMoveToAngle{
    public ArmMoveToL2Back(){
        super(Settings.Arm.L2_ANGLE_BACK);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}
