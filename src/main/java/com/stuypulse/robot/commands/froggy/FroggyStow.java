package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

public class FroggyStow extends FroggySetPivot {
    
    public FroggyStow() {
        super(Settings.Froggy.STOW_ANGLE);
    }
    public void initialize(){
        super.initialize();
    }

    @Override
    public void end(boolean interrupted){
        froggy.stopRoller();
    }

    @Override
    public boolean isFinished(){
        return froggy.getCurrentAngle().equals(Settings.Froggy.STOW_ANGLE);
    }
}