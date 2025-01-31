package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.subsystems.froggy.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FroggySetPivot extends InstantCommand {
    
    protected final Froggy froggy; 
    private final double angle; //target angle

    public FroggySetPivot(double angle) {
        froggy = Froggy.getInstance();

        this.angle = angle;
        addRequirements(froggy);
    }

    @Override
    public void initialize() {
        froggy.setTargetAngle(angle);
    }
}