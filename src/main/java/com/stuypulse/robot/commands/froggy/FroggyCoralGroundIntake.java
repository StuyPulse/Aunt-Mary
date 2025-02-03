package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FroggyCoralGroundIntake extends SequentialCommandGroup{
    
    public FroggyCoralGroundIntake(){
        addCommands(
            new FroggySetPivot(Settings.Froggy.CORAL_GROUND_PICKUP_ANGLE),
            new FroggyIntakeCoral()
        );
    }

    
}