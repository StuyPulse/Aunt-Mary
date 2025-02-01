package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class FroggyProcessorScore extends FroggySetPivot {
    
    public FroggyProcessorScore(){
        super(Settings.Froggy.PROCESSOR_SCORE_ANGLE);
    }

    public void initialize(){
        super.initialize();
        //new WaitUntilCommand(() -> froggy.hasAlgae());
        //froggy.outakeAlgae();
        //move to parallel
    }
}