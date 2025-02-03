package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

public class FroggyAlgaeGroundPickup extends FroggySetPivot {
    
    public FroggyAlgaeGroundPickup(){
        super(Settings.Froggy.ALGAE_GROUND_PICKUP_ANGLE);
    }
    public void initialize(){
        super.initialize();
    }
    
}