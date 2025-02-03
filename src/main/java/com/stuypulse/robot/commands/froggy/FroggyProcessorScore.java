package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FroggyProcessorScore extends SequentialCommandGroup{
    
    public FroggyProcessorScore(){
        addCommands(
            new FroggySetPivot(Settings.Froggy.PROCESSOR_SCORE_ANGLE),
            new FroggyOuttakeAlgae()
        );
    }

    
}