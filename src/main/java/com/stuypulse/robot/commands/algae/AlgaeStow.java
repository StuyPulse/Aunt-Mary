package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeStow extends AlgaeSetPivot {
    
    public AlgaeStow() {
        super(Settings.Algae.STOW_ANGLE);
    }
}