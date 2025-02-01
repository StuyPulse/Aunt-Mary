package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.subsystems.froggy.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FroggySetPivot extends InstantCommand {
    
    protected final Froggy froggy; 
    private final Rotation2d targetAngle;

    public FroggySetPivot(Rotation2d angle) {
        froggy = Froggy.getInstance();
        this.targetAngle = angle;
        addRequirements(froggy);
    }

    @Override
    public void initialize() {
        froggy.setTargetAngle(targetAngle);
    }

    @Override
    public void end(boolean interrupted){
        froggy.stopRoller();
    }

    @Override
    public boolean isFinished(){
        return froggy.getCurrentAngle().equals(targetAngle);
    }
}