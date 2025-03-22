package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Field.CoralStation;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface Clearances {
    public static boolean isArmClearFromReef() {
        return isArmClearFromAllianceReef() && isArmClearFromOppositeAllianceReef();
    }

    private static boolean isArmClearFromOppositeAllianceReef() {
        return Field.OPPOSITE_ALLIANCE_REEF_CENTER.getDistance(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()) 
            > (Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM 
                + Field.CENTER_OF_REEF_TO_REEF_FACE 
                + Constants.LENGTH_WITH_BUMPERS_METERS / 2 
                - Math.hypot(Alignment.Tolerances.X_TOLERANCE, Alignment.Tolerances.Y_TOLERANCE));
    }

    private static boolean isArmClearFromAllianceReef() {
        return Field.ALLIANCE_REEF_CENTER.getDistance(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()) 
            > (Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM 
                + Field.CENTER_OF_REEF_TO_REEF_FACE 
                + Constants.LENGTH_WITH_BUMPERS_METERS / 2 
                - Math.hypot(Alignment.Tolerances.X_TOLERANCE, Alignment.Tolerances.Y_TOLERANCE));
    }

    public static boolean canMoveFroggyWithoutColliding(PivotState targetState) {
        return isFroggyClearFromAllObstables() 
            || Froggy.getInstance().getCurrentAngle().getDegrees() >= 0 && targetState.getTargetAngle().getDegrees() >= 0
            || Froggy.getInstance().getCurrentAngle().getDegrees() <= 0 && targetState.getTargetAngle().getDegrees() <= 0;
    }

    public static boolean isFroggyClearFromAllObstables() {
        return isFroggyClearFromCoralStations() 
            && isFroggyClearFromFieldWalls() 
            && isFroggyClearFromAllianceReef()
            && isFroggyClearFromOppositeAllianceReef();
    }

    private static boolean isFroggyClearFromCoralStations() {
        for (CoralStation coralStation : Field.CoralStation.values()) {
            if (coralStation.getDistanceToStation() < (Constants.WIDTH_WITH_BUMPERS_METERS / 2 + Settings.Clearances.CLEARANCE_DISTANCE_FROGGY)
                && CommandSwerveDrivetrain.getInstance().isFroggyFacingCoralStation(coralStation)) 
            {
                return false;
            }
        }
        return true;
    }

    private static boolean isFroggyClearFromFieldWalls() {
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        Rotation2d froggyHeading = robotPose.getRotation().rotateBy(Rotation2d.kCW_90deg);
        Vector2D froggyHeadingAsVector = new Vector2D(froggyHeading.getCos(), froggyHeading.getSin());

        // Alliance driver station wall
        if (froggyHeadingAsVector.dot(new Vector2D(1, 0)) < 0 && robotPose.getX() - Constants.WIDTH_WITH_BUMPERS_METERS / 2 < Settings.Clearances.CLEARANCE_DISTANCE_FROGGY){
            return false;
        }

        // Opposite driver station wall
        if (froggyHeadingAsVector.dot(new Vector2D(-1, 0)) < 0 && Field.LENGTH - robotPose.getX() - Constants.WIDTH_WITH_BUMPERS_METERS / 2 < Settings.Clearances.CLEARANCE_DISTANCE_FROGGY){
            return false;
        }

        // Left field wall (from perspective of alliance driver station)
        if (froggyHeadingAsVector.dot(new Vector2D(0, -1)) < 0 && Field.WIDTH - robotPose.getY() - Constants.WIDTH_WITH_BUMPERS_METERS / 2 < Settings.Clearances.CLEARANCE_DISTANCE_FROGGY){
            return false;
        }

        // Right field wall (from perspective of alliance driver station)
        if (froggyHeadingAsVector.dot(new Vector2D(0, 1)) < 0 && robotPose.getY() - Constants.WIDTH_WITH_BUMPERS_METERS / 2 < Settings.Clearances.CLEARANCE_DISTANCE_FROGGY){
            return false;
        }

        return true;
    }

    private static boolean isFroggyClearFromAllianceReef() {
        return Field.ALLIANCE_REEF_CENTER.getDistance(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()) 
            > (Settings.Clearances.CLEARANCE_DISTANCE_FROGGY 
                + Field.CENTER_OF_REEF_TO_REEF_FACE 
                + Constants.WIDTH_WITH_BUMPERS_METERS / 2)
            || !CommandSwerveDrivetrain.getInstance().isFroggyFacingAllianceReef();
    }

    private static boolean isFroggyClearFromOppositeAllianceReef() {
        return Field.OPPOSITE_ALLIANCE_REEF_CENTER.getDistance(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()) 
            > (Settings.Clearances.CLEARANCE_DISTANCE_FROGGY 
                + Field.CENTER_OF_REEF_TO_REEF_FACE 
                + Constants.WIDTH_WITH_BUMPERS_METERS / 2)
            || !CommandSwerveDrivetrain.getInstance().isFroggyFacingOppositeAllianceReef();
    }
}
