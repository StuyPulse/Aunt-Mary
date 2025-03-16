package com.stuypulse.robot.commands.swerve.pathFindToPose;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePathFindToPose extends Command{
    private Supplier<Pose2d> targetPose;
    private Command lastPathFindCommand;
    private StopWatch stopWatch;

    public SwerveDrivePathFindToPose(Supplier<Pose2d> targetPose) {
        this.targetPose = targetPose;
        this.stopWatch = new StopWatch();
    }

    public SwerveDrivePathFindToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public static SwerveDrivePathFindToPose pathFindToNearestCoralStation() {
        return new SwerveDrivePathFindToPose(() -> Field.CoralStation.getClosestCoralStation().getTargetPose());
    }

    @Override
    public void initialize() {
        lastPathFindCommand = AutoBuilder.pathfindToPose(targetPose.get(), Settings.Swerve.Constraints.DEFAULT_CONSTRAINTS);
        lastPathFindCommand.initialize();
        stopWatch.reset();
    }

    @Override
    public void execute() {
        if (stopWatch.getTime() > 0.5) {
            lastPathFindCommand.end(false);
            lastPathFindCommand = AutoBuilder.pathfindToPose(targetPose.get(), Settings.Swerve.Constraints.DEFAULT_CONSTRAINTS);
            lastPathFindCommand.initialize();
            stopWatch.reset();
        }
        lastPathFindCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return lastPathFindCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        lastPathFindCommand.end(interrupted);
    }
}
