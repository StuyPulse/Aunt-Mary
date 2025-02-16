package com.stuypulse.robot.constants;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.AprilTag;

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

    public static boolean isValidAprilTagId(int id) {
        return id >= 1 && id <= 22;
    }

    /*** REEF POSITIONS ***/

    Translation2d REEF_CENTER = new Translation2d(Units.inchesToMeters(144.0 + 93.5 / 2), WIDTH / 2);
    double CENTER_OF_TROUGH_TO_BRANCH = Units.inchesToMeters(13.0/2.0);

    public enum CoralBranch {
        A,B,C,D,E,F,G,H,I,J,K,L;

        public Pose2d getTargetPose(int level, boolean isScoringFrontSide) {
            Pose3d correspondingAprilTagPose;
            switch (this) {
                case A, B:
                    correspondingAprilTagPose = Robot.isBlue() ? NamedTags.BLUE_AB.getLocation() : NamedTags.RED_AB.getLocation();
                    break;
                case C, D:
                    correspondingAprilTagPose = Robot.isBlue() ? NamedTags.BLUE_CD.getLocation() : NamedTags.RED_CD.getLocation();
                    break;
                case E, F:
                    correspondingAprilTagPose = Robot.isBlue() ? NamedTags.BLUE_EF.getLocation() : NamedTags.RED_EF.getLocation();
                    break;
                case G, H:
                    correspondingAprilTagPose = Robot.isBlue() ? NamedTags.BLUE_GH.getLocation() : NamedTags.RED_GH.getLocation();
                    break;
                case I, J:
                    correspondingAprilTagPose = Robot.isBlue() ? NamedTags.BLUE_IJ.getLocation() : NamedTags.RED_IJ.getLocation();
                    break;
                case K, L:
                default:
                    correspondingAprilTagPose = Robot.isBlue() ? NamedTags.BLUE_KL.getLocation() : NamedTags.RED_KL.getLocation();
                    break;
            }

            double targetDistanceFromReef;

            switch (level) {
                case 2:
                    targetDistanceFromReef = isScoringFrontSide ? Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L2_FRONT : Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L2_BACK;
                    break;
                case 3:
                    targetDistanceFromReef = isScoringFrontSide ? Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L3_FRONT : Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L3_BACK;
                    break;
                case 4:
                    targetDistanceFromReef = isScoringFrontSide ? Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L4_FRONT : Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L4_BACK;
                    break;
                default:
                    throw new IllegalArgumentException("Branch level provided to CoralBranch.getTargetPose() was invalid. Should be in range [2,4]");
            }

            return correspondingAprilTagPose.toPose2d().transformBy(
                new Transform2d(
                    Constants.LENGTH_WITH_BUMPERS_METERS/2 + targetDistanceFromReef, 
                    CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftPeg() ? -1 : 1) + Constants.SHOOTER_Y_OFFSET, 
                    isScoringFrontSide ? Rotation2d.k180deg : Rotation2d.kZero));
        }

        public boolean isLeftPeg() {
            return switch (this) {
                case A, C, E, G, I, K -> true;
                default -> false;
            };
        }

        /**  
         * Represents the half of the reef facing the driver station.
         * The other half is facing the barge / the opposite alliance
        */
        public boolean onDriverStationSide() {
            return switch (this) {
                case A, B, C, D, L, K -> true;
                default -> false;
            };
        }
    }

    public static CoralBranch getClosestBranch() {
        CoralBranch nearestBranch = CoralBranch.A;
        double closestDistance = Double.MAX_VALUE;

        for (CoralBranch branch : CoralBranch.values()) {
            // just pick a random level for the branch (in this case 2) since it doesnt really matter
            double distance = CommandSwerveDrivetrain.getInstance().getPose().minus(branch.getTargetPose(2, true)).getTranslation().getNorm();
            if (distance < closestDistance) {
                closestDistance = distance;
                nearestBranch = branch;
            }
        }

        return nearestBranch;
    }

    /*** REEF ALGAE ***/

    public enum Algae {
        AB,CD,EF,GH,IJ,KL;

        public boolean isHighAlgae() {
            return switch (this) {
                case AB, EF, IJ -> true;
                default -> false;
            };
        }

        public Pose2d getTargetPose() {
            Pose3d correspondingAprilTag;
            switch (this) {
                case AB:
                    correspondingAprilTag = Robot.isBlue() ? NamedTags.BLUE_AB.getLocation() : NamedTags.RED_AB.getLocation();
                    break;
                case CD:
                    correspondingAprilTag = Robot.isBlue() ? NamedTags.BLUE_CD.getLocation() : NamedTags.RED_CD.getLocation();
                    break;
                case EF:
                    correspondingAprilTag = Robot.isBlue() ? NamedTags.BLUE_EF.getLocation() : NamedTags.RED_EF.getLocation();
                    break;
                case GH:
                    correspondingAprilTag = Robot.isBlue() ? NamedTags.BLUE_GH.getLocation() : NamedTags.RED_GH.getLocation();
                    break;
                case IJ:
                    correspondingAprilTag = Robot.isBlue() ? NamedTags.BLUE_IJ.getLocation() : NamedTags.RED_IJ.getLocation();
                    break;
                case KL:
                default:
                    correspondingAprilTag = Robot.isBlue() ? NamedTags.BLUE_KL.getLocation() : NamedTags.RED_KL.getLocation();
                    break;
            }
            return correspondingAprilTag.toPose2d()
                    .transformBy(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2, 0, Rotation2d.fromDegrees(180)));
        }
    }

    public static Algae getClosestAlgaeBranch() {
        Algae nearestAlgaeBranch = Algae.AB;
        double closestDistance = Double.MAX_VALUE;

        for (Algae algaeBranch : Algae.values()) {
            double distance = CommandSwerveDrivetrain.getInstance().getPose().minus(algaeBranch.getTargetPose()).getTranslation().getNorm();
            if (distance < closestDistance) {
                closestDistance = distance;
                nearestAlgaeBranch = algaeBranch;
            }
        }

        return nearestAlgaeBranch;
    }

    /*** CORAL STATIONS ***/
    public static Pose2d getTargetPoseForCDCoralStation() {
        return Robot.isBlue()
            ? NamedTags.BLUE_CD_CORAL_STATION.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2, 0, new Rotation2d()))
            : NamedTags.RED_CD_CORAL_STATION.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2, 0, new Rotation2d()));
    }

    public static Pose2d getTargetPoseForKLCoralStation() {
        return Robot.isBlue()
            ? NamedTags.BLUE_KL_CORAL_STATION.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2, 0, new Rotation2d()))
            : NamedTags.RED_KL_CORAL_STATION.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2, 0, new Rotation2d()));
    }

    public static Pose2d getClosestCoralStationTargetPose() {
        Pose2d robot = CommandSwerveDrivetrain.getInstance().getPose();

        Pose2d cdCoralStation = getTargetPoseForCDCoralStation();
        Pose2d klCoralStation = getTargetPoseForKLCoralStation();

        if (robot.minus(cdCoralStation).getTranslation().getNorm() < robot.minus(klCoralStation).getTranslation().getNorm()) {
            return cdCoralStation;
        }
        else {
            return klCoralStation;
        }
    }

    /**** EMPTY FIELD POSES ****/

    Pose2d EMPTY_FIELD_POSE2D = new Pose2d(new Translation2d(-1, -1), new Rotation2d());
    Pose3d EMPTY_FIELD_POSE3D = new Pose3d(-1, -1, 0, new Rotation3d());

    public static void clearFieldObject(FieldObject2d fieldObject)  {
        fieldObject.setPose(EMPTY_FIELD_POSE2D);
    }
}