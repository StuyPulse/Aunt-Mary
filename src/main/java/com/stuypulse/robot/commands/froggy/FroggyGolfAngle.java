package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

public class FroggyGolfAngle extends FroggySetPivot{

    public FroggyGolfAngle(){
        super(Settings.Froggy.GOLF_TEE_ALGAE_PICKUP_ANGLE);
    }
    public void initialize(){
        super.initialize();
    }
}