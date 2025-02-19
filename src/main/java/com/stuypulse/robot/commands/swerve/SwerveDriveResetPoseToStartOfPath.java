package com.stuypulse.robot.commands.swerve;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.PathUtil;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveResetPoseToStartOfPath extends InstantCommand{
    private final CommandSwerveDrivetrain swerve;
    private final PathPlannerPath path;

    public SwerveDriveResetPoseToStartOfPath(PathPlannerPath path) {
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.path = path;
    }

    public SwerveDriveResetPoseToStartOfPath(String path) {
        this(PathUtil.load(path));
    }

    @Override
    public void initialize() {
        swerve.resetPose(path.getStartingDifferentialPose());
    }
}
