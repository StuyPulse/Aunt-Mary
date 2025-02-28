package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetIMUMode extends InstantCommand{
    private final LimelightVision vision;
    private final int mode;

    public VisionSetIMUMode(int mode) {
        this.vision = LimelightVision.getInstance();
        this.mode = mode;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.setIMUMode(mode);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
