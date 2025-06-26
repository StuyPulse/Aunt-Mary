package com.stuypulse.robot.commands.swerve.pathFindToPose;

import java.util.ArrayList;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.ReefFace;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveDynamicObstacles{
    public static InstantCommand reset() {
        ArrayList<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
        return new InstantCommand(() -> Pathfinding.setDynamicObstacles(obstacles, CommandSwerveDrivetrain.getInstance().getPose().getTranslation()));
    }

    public static InstantCommand reefClearance() {
        ArrayList<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
        for (ReefFace reefFace : ReefUtil.ReefFace.values()) {
            Pose2d correspondingAprilTag = reefFace.getCorrespondingAprilTagPose();

            // Left and right of the reef face
            Pose2d reefFaceCorner1 = correspondingAprilTag.transformBy(new Transform2d(0, Field.LENGTH_OF_REEF_FACE / 2, Rotation2d.kZero));
            Pose2d reefFaceCorner2 = correspondingAprilTag.transformBy(new Transform2d(0, -Field.LENGTH_OF_REEF_FACE / 2, Rotation2d.kZero));

            int boundingBoxesPerCorner = 5; // increase steps for more bounding boxes. Its the same area covered but less space in between each one.
            Translation2d transformFromCenterToCornerOfBoundingBox = new Translation2d(Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM * Math.sqrt(2), Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM * Math.sqrt(2));

            // reef face corner 1
            for (int i = 0; i < boundingBoxesPerCorner; i++) {
                Translation2d centerOfBoundingBox = reefFaceCorner1.transformBy(new Transform2d(Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM * i / boundingBoxesPerCorner, 0, Rotation2d.kZero)).getTranslation();
                Translation2d boundingBoxCorner1 = centerOfBoundingBox.plus(transformFromCenterToCornerOfBoundingBox);
                Translation2d boundingBoxCorner2 = centerOfBoundingBox.minus(transformFromCenterToCornerOfBoundingBox);

                Pair<Translation2d, Translation2d> boundingBox = new Pair<Translation2d,Translation2d>(boundingBoxCorner1, boundingBoxCorner2);
                obstacles.add(boundingBox);
            }

            // reef face corner 2
            for (int i = 0; i < boundingBoxesPerCorner; i++) {
                Translation2d centerOfBoundingBox = reefFaceCorner2.transformBy(new Transform2d(Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM * i / boundingBoxesPerCorner, 0, Rotation2d.kZero)).getTranslation();
                Translation2d boundingBoxCorner1 = centerOfBoundingBox.plus(transformFromCenterToCornerOfBoundingBox);
                Translation2d boundingBoxCorner2 = centerOfBoundingBox.minus(transformFromCenterToCornerOfBoundingBox);

                Pair<Translation2d, Translation2d> boundingBox = new Pair<Translation2d,Translation2d>(boundingBoxCorner1, boundingBoxCorner2);
                obstacles.add(boundingBox);
            }
        }
        return new InstantCommand(() -> Pathfinding.setDynamicObstacles(obstacles, CommandSwerveDrivetrain.getInstance().getPose().getTranslation()));
    }
}
