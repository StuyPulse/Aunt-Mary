package com.stuypulse.robot.constants;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.AprilTag;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

/** This interface stores information about the field elements. */
public interface Field {

    public static final Field2d FIELD2D = new Field2d();

    double WIDTH = Units.inchesToMeters(317.000); 
    double LENGTH = Units.inchesToMeters(690.876);

    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
            rotated.getTranslation().plus(new Translation3d(LENGTH, WIDTH, 0)),
            rotated.getRotation());
    }

    public static Pose2d transformToOppositeAlliance(Pose2d pose) {
        Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(
            rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)),
            rotated.getRotation());
    }

    /*** APRILTAGS ***/

    enum NamedTags {
        RED_KL_CORAL_STATION,
        RED_CD_CORAL_STATION,
        RED_PROCESSOR,
        BLUE_BARGE_RED_SIDE,
        RED_BARGE_RED_SIDE,
        RED_KL,
        RED_AB,
        RED_CD,
        RED_EF,
        RED_GH,
        RED_IJ,
        BLUE_CD_CORAL_STATION,
        BLUE_KL_CORAL_STATION,
        BLUE_BARGE_BLUE_SIDE,
        RED_BARGE_BLUE_SIDE,
        BLUE_PROCESSOR,
        BLUE_CD,
        BLUE_AB,
        BLUE_KL,
        BLUE_IJ,
        BLUE_GH,
        BLUE_EF;

        public final AprilTag tag;

        public int getID() {
            return tag.getID();
        }

        public Pose3d getLocation() {
            return Robot.isBlue()
                ? tag.getLocation()
                : transformToOppositeAlliance(tag.getLocation());
        }

        private NamedTags() {
            tag = APRILTAGS[ordinal()];
        }
    }

    AprilTag APRILTAGS[] = {
        // 2025 Field AprilTag Layout
        new AprilTag(1,  new Pose3d(new Translation3d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(126)))),
        new AprilTag(2,  new Pose3d(new Translation3d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(234)))),
        new AprilTag(3,  new Pose3d(new Translation3d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15), Units.inchesToMeters(51.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(4,  new Pose3d(new Translation3d(Units.inchesToMeters(365.20), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(5,  new Pose3d(new Translation3d(Units.inchesToMeters(365.20), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(6,  new Pose3d(new Translation3d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
        new AprilTag(7,  new Pose3d(new Translation3d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(8,  new Pose3d(new Translation3d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new AprilTag(9,  new Pose3d(new Translation3d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new AprilTag(10,  new Pose3d(new Translation3d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(11,  new Pose3d(new Translation3d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
        new AprilTag(12,  new Pose3d(new Translation3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(54)))),
        new AprilTag(13,  new Pose3d(new Translation3d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(306)))),
        new AprilTag(14,  new Pose3d(new Translation3d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(15,  new Pose3d(new Translation3d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54)), new Rotation3d(Units.degreesToRadians(30), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(16,  new Pose3d(new Translation3d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15), Units.inchesToMeters(51.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(17,  new Pose3d(new Translation3d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
        new AprilTag(18,  new Pose3d(new Translation3d(Units.inchesToMeters(144.0), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(19,  new Pose3d(new Translation3d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new AprilTag(20,  new Pose3d(new Translation3d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new AprilTag(21,  new Pose3d(new Translation3d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(22,  new Pose3d(new Translation3d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
    };

    public static boolean isValidTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return true;
            }
        }
        return false;
    }

    public static AprilTag getTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return tag;
            }
        }
        return null;
    }

    /*** REEF POSITIONS ***/
    Translation2d REEF_CENTER = new Translation2d(Units.inchesToMeters(144.0 + (93.5 - 14.0 * 2) / 2), Field.WIDTH / 2);
    double CENTER_OF_REEF_TO_REEF_FACE = Units.inchesToMeters(32.75);
    double CENTER_OF_TROUGH_TO_BRANCH = Units.inchesToMeters(13.0/2.0);

    /*** BARGE POSITIONS ***/
    public static Pose2d getBargeTargetPose(Pose2d robot) {
        if (Robot.isBlue()) {
            return new Pose2d(new Translation2d(
                Field.LENGTH / 2 - Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE, 
                Field.WIDTH / 2 + Settings.Swerve.Alignment.Targets.HORIZONTAL_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO), 
                NamedTags.BLUE_BARGE_BLUE_SIDE.getLocation().getRotation().toRotation2d());
        } else {
            return new Pose2d(new Translation2d(
                Field.LENGTH / 2 - Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE, 
                Field.WIDTH / 2 + Settings.Swerve.Alignment.Targets.HORIZONTAL_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO), 
                NamedTags.RED_BARGE_RED_SIDE.getLocation().getRotation().toRotation2d());
        }
    }

    /*** PROCESSOR ***/
    public static Pose2d getTargetPoseForProcessorShooter() {
        return Robot.isBlue()
            ? NamedTags.BLUE_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2 - 0.08, Constants.SHOOTER_Y_OFFSET, Rotation2d.k180deg))
            : NamedTags.RED_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2 - 0.08, Constants.SHOOTER_Y_OFFSET, Rotation2d.k180deg));
    }

    public static Pose2d getTargetPoseForProcessorFroggy() {
        return Robot.isBlue()
            ? NamedTags.BLUE_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.WIDTH_WITH_BUMPERS_METERS / 2, 0, Rotation2d.kCW_90deg))
            : NamedTags.RED_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.WIDTH_WITH_BUMPERS_METERS / 2, 0, Rotation2d.kCW_90deg));
    }

    /*** CORAL STATIONS ***/

    /*** CORAL STATIONS ***/

    public enum CoralStation {
        BLUE_CD_CORAL_STATION(NamedTags.BLUE_CD_CORAL_STATION, new Translation2d(0, Field.WIDTH / 2 - Units.inchesToMeters(109.13)), new Translation2d(Units.inchesToMeters(65.84), 0)),
        BLUE_KL_CORAL_STATION(NamedTags.BLUE_KL_CORAL_STATION, new Translation2d(0, Field.WIDTH / 2 + Units.inchesToMeters(109.13)), new Translation2d(Units.inchesToMeters(65.84), Field.WIDTH)),
        RED_CD_CORAL_STATION(NamedTags.RED_CD_CORAL_STATION, new Translation2d(Field.LENGTH, Field.WIDTH / 2 + Units.inchesToMeters(109.13)), new Translation2d(Field.LENGTH - Units.inchesToMeters(65.84), Field.WIDTH)),
        RED_KL_CORAL_STATION(NamedTags.RED_KL_CORAL_STATION, new Translation2d(Field.LENGTH, Field.WIDTH / 2 - Units.inchesToMeters(109.13)), new Translation2d(Field.LENGTH - Units.inchesToMeters(65.84), 0));
    
        private NamedTags correspondingAprilTag;
        private Translation2d blueOriginLineStart; // Driver station wall
        private Translation2d blueOriginLineEnd; // Side of field

        private CoralStation(NamedTags correspondingAprilTag, Translation2d blueOriginLineStart, Translation2d blueOriginLineEnd) {
            this.correspondingAprilTag = correspondingAprilTag;
            this.blueOriginLineStart = blueOriginLineStart;
            this.blueOriginLineEnd = blueOriginLineEnd;
        }

        private Translation2d getLineStart() {
            return Robot.isBlue()
                ? blueOriginLineStart
                : transformToOppositeAlliance(new Pose2d(blueOriginLineStart, Rotation2d.kZero)).getTranslation();
        }

        private Translation2d getLineEnd() {
            return Robot.isBlue()
                ? blueOriginLineEnd
                : transformToOppositeAlliance(new Pose2d(blueOriginLineEnd, Rotation2d.kZero)).getTranslation();
        }

        // Kalimul said to add a comment that this is getting the closest point to a line
        public Pose2d getTargetPose() {
            Translation2d lineStart = getLineStart();
            Translation2d lineEnd = getLineEnd();
            Translation2d robot = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();

            Vector2D lineStartToEnd = new Vector2D(lineEnd.getX() - lineStart.getX(), lineEnd.getY() - lineStart.getY());
            Vector2D lineStartToPoint = new Vector2D(robot.getX() - lineStart.getX(), robot.getY() - lineStart.getY());
            
            double lineLengthSquared = lineStartToEnd.dot(lineStartToEnd);
            double dotProduct = lineStartToEnd.dot(lineStartToPoint);
            
            double t = dotProduct / lineLengthSquared; // Projection factor
            
            double coralStationLength = lineStart.getDistance(lineEnd);
            double percentToIgnoreFromEachSide = (Constants.WIDTH_WITH_BUMPERS_METERS / 2) / coralStationLength;
            
            t = Math.max(percentToIgnoreFromEachSide, Math.min(1 - percentToIgnoreFromEachSide, t));
            
            Translation2d closestPointOnCoralStation = new Translation2d(lineStart.getX() + t * lineStartToEnd.x, lineStart.getY() + t * lineStartToEnd.y);

            return new Pose2d(closestPointOnCoralStation, correspondingAprilTag.getLocation().toPose2d().getRotation()).transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS / 2 + Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION, 
                0, Rotation2d.kZero));
        }

        public Vector2D getHeadingAsVector() {
            return new Vector2D(
                Math.cos(correspondingAprilTag.getLocation().getRotation().getZ()), 
                Math.sin(correspondingAprilTag.getLocation().getRotation().getZ()));
        }

        // https://www.youtube.com/watch?v=KHuI9bXZS74
        public double getDistanceToStation() {
            Vector2D A = new Vector2D(getLineStart());
            Vector2D B = new Vector2D(getLineEnd());
            Vector2D C = new Vector2D(CommandSwerveDrivetrain.getInstance().getPose().getTranslation());
            
            return Math.abs((C.x - A.x) * (-B.y + A.y) + (C.y - A.y) * (B.x - A.x))
                / Math.sqrt(Math.pow((-B.y + A.y), 2) + Math.pow((B.x - A.x), 2));
        }

        public static CoralStation getClosestCoralStation() {
            if (Robot.isBlue()) {
                return BLUE_CD_CORAL_STATION.getDistanceToStation() <  BLUE_KL_CORAL_STATION.getDistanceToStation()
                    ? BLUE_CD_CORAL_STATION
                    : BLUE_KL_CORAL_STATION;
            }
            else {
                return RED_CD_CORAL_STATION.getDistanceToStation() <  RED_KL_CORAL_STATION.getDistanceToStation()
                    ? RED_CD_CORAL_STATION
                    : RED_KL_CORAL_STATION;
            }
        }

        public static double getDistanceToClosestStation(Pose2d pose) {
            return getClosestCoralStation().getDistanceToStation();
        }

        public static CoralStation getCoralStation(boolean isCD) {
            if (isCD) {
                return Robot.isBlue() ?
                    CoralStation.BLUE_CD_CORAL_STATION : 
                    CoralStation.RED_CD_CORAL_STATION; 
            } else {
                return Robot.isBlue() ?
                    CoralStation.BLUE_KL_CORAL_STATION :
                    CoralStation.RED_KL_CORAL_STATION;
            }
        }
    }

    /**** EMPTY FIELD POSES ****/

    Pose2d EMPTY_FIELD_POSE2D = new Pose2d(new Translation2d(-1, -1), new Rotation2d());
    Pose3d EMPTY_FIELD_POSE3D = new Pose3d(-1, -1, 0, new Rotation3d());

    public static void clearFieldObject(FieldObject2d fieldObject)  {
        fieldObject.setPose(EMPTY_FIELD_POSE2D);
    }
}