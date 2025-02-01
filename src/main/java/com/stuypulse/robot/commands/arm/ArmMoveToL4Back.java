package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmMoveToL4Back extends ArmMoveToAngle {
    public ArmMoveToL4Back(){
        super(Settings.Arm.L4_ANGLE_BACK);
    }
    @Override
    public void initialize(){
        super.initialize();
    }
}
