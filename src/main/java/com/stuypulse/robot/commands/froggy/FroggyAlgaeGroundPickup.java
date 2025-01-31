package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

public class FroggyAlgaeGroundPickup extends FroggySetPivot {
    
    public FroggyAlgaeGroundPickup(){
        super(Settings.Froggy.ALGAE_GROUND_PICKUP_ANGLE);
        froggy.intakeAlgae();
    }
    public void initialize(){
        super.initialize();
    }
     
}