package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

public class FroggyAlgaeGroundAngle extends FroggySetPivot {
    
    public FroggyAlgaeGroundAngle(){
        super(Settings.Froggy.ALGAE_GROUND_PICKUP_ANGLE);
        froggy.intakeAlgae();
    }
    public void initialize(){
        super.initialize();
    }
     
}