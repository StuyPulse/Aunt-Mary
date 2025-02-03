package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FroggyScoreL1 extends SequentialCommandGroup{
    
    public FroggyScoreL1(){
        addCommands(
            new FroggySetPivot(Settings.Froggy.L1_SCORING_ANGLE),
            new FroggyOuttakeCoral()
        );
    }

    
}
