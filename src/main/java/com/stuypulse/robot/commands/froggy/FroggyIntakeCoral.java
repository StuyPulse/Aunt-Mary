package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.subsystems.froggy.*;
import edu.wpi.first.wpilibj2.command.Command;

public class FroggyIntakeCoral extends Command {
    
    protected final Froggy froggy; 

    public FroggyIntakeCoral() {
        froggy = Froggy.getInstance();
        addRequirements(froggy);
    }

    @Override
    public void execute() {
        froggy.intakeCoral();
    }

    @Override
    public void end(boolean interrupted){
        froggy.stopRoller();
    }

    @Override
    public boolean isFinished(){
        return froggy.hasCoral();
    }
}