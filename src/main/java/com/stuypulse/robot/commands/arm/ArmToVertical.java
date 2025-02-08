package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToVertical extends ArmToAngle{
    public ArmToVertical(){
        super(Settings.Arm.VERTICAL_ANGLE);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(false);
        super.initialize();
    }
}