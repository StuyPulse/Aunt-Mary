package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelStop extends InstantCommand{
    private final Funnel funnel;

    public FunnelStop() {
        this.funnel = Funnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.setState(FunnelState.STOP);
    }
}
