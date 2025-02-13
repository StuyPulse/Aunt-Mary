package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.climb.Climb;

public class ManualClimb extends Command {
    private final Climb climb;
    private final double speed;
    public ManualClimb(double speed){
        climb = Climb.getInstance();
        this.speed = speed;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.setDutyCycle(speed);
    }
}