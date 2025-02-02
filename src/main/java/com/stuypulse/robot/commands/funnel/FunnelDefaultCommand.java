package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.CoralFunnel;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FunnelDefaultCommand extends Command {
    private final CoralFunnel funnel;
    private boolean stopped = false;
    private boolean reversed = false;

    public FunnelDefaultCommand() {
        funnel = CoralFunnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        setState();

        runState();
    }

    private void setState(){
        stopped = LokiShooter.getInstance().hasCoral();
        if(funnel.isStalling() && !reversed) {
            reversed = true;

            new WaitCommand(1).andThen(() -> {
                reversed = false;
            }).schedule();
        }
    }

    private void runState(){
        if (stopped){
            funnel.stop();
        } else if (reversed){
            funnel.reverse();
        } else {
            funnel.forward();
        }
    }
}
