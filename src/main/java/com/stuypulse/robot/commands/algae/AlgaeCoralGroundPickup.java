package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeCoralGroundPickup extends AlgaeSetPivot {
    
    public AlgaeCoralGroundPickup(){
        super(Settings.Algae.CORAL_GROUND_PICKUP_ANGLE);
        algae.intakeCoral();
    }
    public void initialize(){
        super.initialize();
    }
     
}