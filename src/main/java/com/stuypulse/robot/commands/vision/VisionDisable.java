package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionDisable extends InstantCommand{
    private final LimelightVision vision;

    public VisionDisable() {
        this.vision = LimelightVision.getInstance();
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        Settings.EnabledSubsystems.VISION.set(false);
    }
}
