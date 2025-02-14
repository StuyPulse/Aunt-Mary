package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.subsystems.froggy.*;

import edu.wpi.first.wpilibj2.command.Command;


public class FroggyManual extends Command{
    private final Froggy froggy;
    private final double speed;

    public FroggyManual(double speed){
        froggy = Froggy.getInstance();
        this.speed = speed;
        addRequirements(froggy);
    }

    @Override
    public void execute() {
        froggy.setDutyCycle(speed);
    }
}
