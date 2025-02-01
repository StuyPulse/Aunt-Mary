package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FroggyL1 extends FroggySetPivot {
    
    public FroggyL1() {
        super(Settings.Froggy.L1_SCORING_ANGLE);
    }
    public void initialize(){
        super.initialize();
        //new WaitUntilCommand(() -> froggy.hasCoral());
        //froggy.outakeCoral();
        //move to parallel command group
    }
}
