package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmMoveToL3Back extends ArmMoveToAngle {
    public ArmMoveToL3Back(){
        super(Settings.Arm.L3_ANGLE_BACK);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}
