package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeGroundPickup extends AlgaeSetPivot {
    
    public AlgaeGroundPickup(){
        super(Settings.Algae.ALGAE_GROUND_PICKUP_ANGLE);
        algae.intakeAlgae();
    }
    public void initialize(){
        super.initialize();
    }
     
}