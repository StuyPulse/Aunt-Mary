package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

public class FroggyGolf extends FroggySetPivot{

    public FroggyGolf(double angle){
        super(Settings.Froggy.GOLF_TEE_ALGAE_PICKUP_ANGLE);
        froggy.intakeAlgae();
    }
    public void initialize(){
        super.initialize();
    }
}