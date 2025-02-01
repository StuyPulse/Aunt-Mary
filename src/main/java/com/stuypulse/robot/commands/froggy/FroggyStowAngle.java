package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

public class FroggyStowAngle extends FroggySetPivot {
    
    public FroggyStowAngle() {
        super(Settings.Froggy.STOW_ANGLE);
    }
    public void initialize(){
        super.initialize();
    }
}