package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.froggy.Froggy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class FroggyStow extends FroggySetPivot {
    
    public FroggyStow() {
        super(Settings.Froggy.STOW_ANGLE);
    }
    
}

