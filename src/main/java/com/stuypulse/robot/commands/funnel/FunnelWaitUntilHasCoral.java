package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FunnelWaitUntilHasCoral extends WaitUntilCommand{
    public FunnelWaitUntilHasCoral() {
        super(() -> Funnel.getInstance().hasCoral());
    }
}
