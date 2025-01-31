package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeGolf extends AlgaeSetPivot{

    public AlgaeGolf(double angle){
        super(Settings.Algae.GOLF_TEE_ALGAE_PICKUP_ANGLE);
        algae.intakeAlgae();
    }
}