package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag1;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveSeedFieldRelativeTest extends InstantCommand{
    private final CommandSwerveDrivetrain swerve;
    private final LimelightVision vision;

    public SwerveDriveSeedFieldRelativeTest() {
        swerve = CommandSwerveDrivetrain.getInstance();
        vision = LimelightVision.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetRotation(vision.getMegaTag1PoseEstimate(Cameras.LimelightCameras[0].getName()).pose.getRotation());
    }
}
