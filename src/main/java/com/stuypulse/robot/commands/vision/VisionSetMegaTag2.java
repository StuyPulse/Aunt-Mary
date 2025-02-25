package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.vision.LimelightVision.MegaTagMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetMegaTag2 extends InstantCommand{
    private final LimelightVision vision;

    public VisionSetMegaTag2() {
        this.vision = LimelightVision.getInstance();
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.setMegaTagMode(MegaTagMode.MEGATAG2);
    }
}
