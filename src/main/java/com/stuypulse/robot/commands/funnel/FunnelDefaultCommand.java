package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.CoralFunnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FunnelDefaultCommand extends Command {
    private final CoralFunnel funnel;
    private boolean reversed = false;

    public FunnelDefaultCommand() {
        funnel = CoralFunnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        setState();

        changeState();
    }

    private void setState(){
        if(funnel.isStalling() && !reversed) {
            reversed = true;

            new WaitCommand(1).andThen(() -> {
                reversed = false;
            }).schedule();
        }
    }

    private void changeState(){
        if (reversed){
            funnel.reverse();
        } else {
            funnel.forward();
        }
    }
}
