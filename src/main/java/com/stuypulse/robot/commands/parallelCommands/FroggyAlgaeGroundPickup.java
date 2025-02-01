package com.stuypulse.robot.commands.parallelCommands;

import com.stuypulse.robot.commands.froggy.FroggyAlgaeGroundAngle;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FroggyAlgaeGroundPickup extends ParallelCommandGroup{
    
    public FroggyAlgaeGroundPickup(){
        addCommands(
            new FroggyAlgaeGroundAngle(),
        );
    }

    
}
