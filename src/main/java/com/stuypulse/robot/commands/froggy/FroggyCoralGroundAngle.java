package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;



public class FroggyCoralGroundAngle extends FroggySetPivot {
    
    public FroggyCoralGroundAngle(){
        super(Settings.Froggy.CORAL_GROUND_PICKUP_ANGLE);
        froggy.intakeCoral();
    } 
    public void initialize(){
        super.initialize();
    }
     
}