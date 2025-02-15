package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelAcquire extends Command{
    
    private final Funnel funnel;
    private final Shooter lokiShooter;

    public FunnelAcquire() {
        this.funnel = Funnel.getInstance();
        this.lokiShooter = Shooter.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        if (lokiShooter.hasCoral()) {
            funnel.setState(FunnelState.STOP);
        }
        else if (funnel.shouldReverse()) {
            funnel.setState(FunnelState.REVERSE);
        }
        else {
            funnel.setState(FunnelState.FORWARD);
        }
    }
}
