package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionEnable extends InstantCommand{
    private final LimelightVision vision;
    private final Camera[] cameras;

    public VisionEnable(Camera... cameras) {
        this.vision = LimelightVision.getInstance();
        this.cameras = cameras;
        addRequirements(vision);
    }

    public VisionEnable() {
        this(Cameras.LimelightCameras);
    }

    @Override
    public void initialize() {
        for (Camera camera : cameras) {
            camera.setEnabled(true);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
