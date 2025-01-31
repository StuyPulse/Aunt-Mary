package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.subsystems.algae.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlgaeSetPivot extends InstantCommand {
    
    protected final Algae algae; 
    private final double angle; //target angle

    public AlgaeSetPivot(double angle) {
        algae = Algae.getInstance();

        this.angle = angle;
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.setTargetAngle(angle);
    }

}