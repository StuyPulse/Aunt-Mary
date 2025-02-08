package com.stuypulse.robot.commands.arm.funnel_side;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.util.Color;

import com.stuypulse.robot.commands.arm.ArmToAngle;
import com.stuypulse.robot.commands.led.LedSolidColor;

public class ArmToL2Funnel extends ArmToAngle{
    public ArmToL2Funnel(){
        super(Settings.Arm.L2_ANGLE_BACK);
    }
    @Override
    public void initialize(){
        arm.setRotateBoolean(true);
        super.initialize();
    }
    @Override
    public void end(boolean interrupted) {
        if (interrupted) new LedSolidColor(Color.kBlue).schedule();
    }
}
