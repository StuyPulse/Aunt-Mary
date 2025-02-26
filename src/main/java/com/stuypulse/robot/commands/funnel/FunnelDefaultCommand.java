package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelDefaultCommand extends Command{
    
    private final Funnel funnel;
    private final Shooter shooter;

    public FunnelDefaultCommand() {
        this.funnel = Funnel.getInstance();
        this.shooter = Shooter.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        if (shooter.hasCoral() || shooter.getState() == ShooterState.HOLD_ALGAE) {
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
