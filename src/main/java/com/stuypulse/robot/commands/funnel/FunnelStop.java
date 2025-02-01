package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.CoralFunnel;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelStop extends InstantCommand {
    private final CoralFunnel funnel;
    public FunnelStop() {
        funnel = CoralFunnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void initialize(){
        funnel.stop();
    }
}
