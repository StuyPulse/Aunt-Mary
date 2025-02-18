package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelSetSpeed extends InstantCommand {
    private final Funnel funnel;
    private final double speed;

    public FunnelSetSpeed(double speed) {
        this.funnel = Funnel.getInstance();
        this.speed = speed;
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.setSpeed(speed);
    }
}
