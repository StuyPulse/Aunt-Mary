package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FroggyGolfIntake extends SequentialCommandGroup{
    
    public FroggyGolfIntake(){
        addCommands(
            new FroggySetPivot(Settings.Froggy.GOLF_TEE_ALGAE_PICKUP_ANGLE),
            new FroggyIntakeAlgae()
        );
    }

    
}
