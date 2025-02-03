package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FroggyAlgaeGroundIntake extends SequentialCommandGroup{
    
    public FroggyAlgaeGroundIntake(){
        addCommands(
            new FroggySetPivot(Settings.Froggy.ALGAE_GROUND_PICKUP_ANGLE),
            new FroggyIntakeAlgae()
        );
    }

    
}
