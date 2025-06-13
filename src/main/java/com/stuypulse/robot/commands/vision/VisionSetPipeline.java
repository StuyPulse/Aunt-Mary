package com.stuypulse.robot.commands.vision;


import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetPipeline extends InstantCommand {

    private final LimelightVision vision;
    private final String limelightString;
    private final int pipeline;

    public VisionSetPipeline(String limelightName, int pipeline) {
        this.limelightString = limelightName;
        this.pipeline = pipeline;
        vision = LimelightVision.getInstance();
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.setPipelineMode(pipeline, limelightString);
    }   

}
