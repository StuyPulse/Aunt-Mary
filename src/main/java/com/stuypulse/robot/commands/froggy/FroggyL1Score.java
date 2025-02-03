package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FroggyL1Score extends SequentialCommandGroup{
    
    public FroggyL1Score(){
        addCommands(
            new FroggySetPivot(Settings.Froggy.L1_SCORING_ANGLE),
            new FroggyOuttakeCoral()
        );
    }

    
}