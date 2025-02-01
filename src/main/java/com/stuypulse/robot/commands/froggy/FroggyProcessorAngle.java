package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;


public class FroggyProcessorAngle extends FroggySetPivot {
    
    public FroggyProcessorAngle(){
        super(Settings.Froggy.PROCESSOR_SCORE_ANGLE);
        
        froggy.outakeAlgae();
    }

    public void initialize(){
        super.initialize();
    }

}