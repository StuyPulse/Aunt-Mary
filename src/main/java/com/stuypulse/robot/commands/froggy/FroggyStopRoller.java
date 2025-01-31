package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.subsystems.froggy.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FroggyStopRoller extends InstantCommand {
    
    private final Froggy froggy; 

    public FroggyStopRoller() {
        froggy = Froggy.getInstance();
        
        addRequirements(froggy);
    }

    @Override
    public void initialize() {
        froggy.stopRoller();
    }

}