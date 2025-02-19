package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelReverse extends Command {
    private final Funnel funnel;

    public FunnelReverse() {
        this.funnel = Funnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.setState(FunnelState.REVERSE);
    }
}
