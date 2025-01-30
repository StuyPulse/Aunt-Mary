package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.CoralFunnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FunnelDefaultCommand extends Command {
    private final CoralFunnel funnel;
    private boolean reversed = false;
    private boolean funnelStopped = false;

    public FunnelDefaultCommand() {
        funnel = CoralFunnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        boolean coralDetected = funnel.getFunnelState();

        if(coralDetected && !funnelStopped) {
            funnelStopped = true;
        }

        if(funnelStopped && !coralDetected) {
            funnelStopped = false;
        }

        if(funnel.coralStuck() && !reversed) {
            reversed = true;

            new WaitCommand(1).andThen(() -> {
                reversed = false;
            }).schedule();
        }

        if (funnelStopped){
            funnel.setMotorRPM(0);
        } else if (reversed){
            funnel.setMotorRPM(-funnel.getTargetRPM());
        } else {
            funnel.setMotorRPM(funnel.getTargetRPM());
        }
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setMotorRPM(0);
    }
}
