package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.vision.LimelightVision.WhitelistMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetTagWhitelist extends InstantCommand{
    private final LimelightVision vision;
    private final WhitelistMode mode;

    public VisionSetTagWhitelist(WhitelistMode mode) {
        this.vision = LimelightVision.getInstance();
        this.mode = mode;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.setWhitelistMode(mode);
    }
}
