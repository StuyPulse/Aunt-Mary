package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelManualReverse extends Command{
    private final Funnel funnel;

    public FunnelManualReverse() {
        funnel = Funnel.getInstance();

        addRequirements(funnel);
    }
    
    public void execute(){
        funnel.reverse();
    }
}
