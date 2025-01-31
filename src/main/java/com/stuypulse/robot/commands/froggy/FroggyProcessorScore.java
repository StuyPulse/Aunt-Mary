package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;


public class FroggyProcessorScore extends FroggySetPivot {
    
    public FroggyProcessorScore(){
        super(Settings.Froggy.PROCESSOR_SCORE_ANGLE);
        
        froggy.outakeAlgae();
    }

    public void initialize(){
        super.initialize();
    }

}