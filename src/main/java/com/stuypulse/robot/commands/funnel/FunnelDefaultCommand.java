package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.CoralFunnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FunnelDefaultCommand extends Command {
    private final CoralFunnel funnel;
    private boolean reversing = false;
    private boolean coralStopped = false;

    public FunnelDefaultCommand() {
        funnel = CoralFunnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        boolean coralDetected = funnel.getFunnelState();

        if(coralDetected) {
            funnel.setMotorRPM(0);
            coralStopped = true;
            return;
        }

        if(coralStopped && !coralDetected) {
            coralStopped = false;
            funnel.setMotorRPM(funnel.getTargetRPM());
        }

        if(funnel.coralStuck() && !reversing) {
            reversing = true;
            funnel.setMotorRPM(-funnel.getTargetRPM());

            new WaitCommand(1).andThen(() -> {
                funnel.setMotorRPM(funnel.getTargetRPM());
                reversing = true;
            }).schedule();
        }

        if(!reversing && !coralStopped) {
            funnel.setMotorRPM(funnel.getTargetRPM());
        }
    }

    @Override
    public void end(boolean interrupted) {
        funnel.setMotorRPM(0);
    }
}
