package com.stuypulse.robot.util;

import java.util.ArrayList;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.NamedTags;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface ReefUtil {

    public enum CoralBranch {
        A(NamedTags.BLUE_AB, NamedTags.RED_AB),
        B(NamedTags.BLUE_AB, NamedTags.RED_AB),
        C(NamedTags.BLUE_CD, NamedTags.RED_CD),
        D(NamedTags.BLUE_CD, NamedTags.RED_CD),
        E(NamedTags.BLUE_EF, NamedTags.RED_EF),
        F(NamedTags.BLUE_EF, NamedTags.RED_EF),
        G(NamedTags.BLUE_GH, NamedTags.RED_GH),
        H(NamedTags.BLUE_GH, NamedTags.RED_GH),
        I(NamedTags.BLUE_IJ, NamedTags.RED_IJ),
        J(NamedTags.BLUE_IJ, NamedTags.RED_IJ),
        K(NamedTags.BLUE_KL, NamedTags.RED_KL),
        L(NamedTags.BLUE_KL, NamedTags.RED_KL);

        private NamedTags correspondingBlueAprilTag;
        private NamedTags correspondingRedAprilTag;

        private CoralBranch(NamedTags correspondingBlueAprilTag, NamedTags correspondingRedAprilTag) {
            this.correspondingBlueAprilTag = correspondingBlueAprilTag;
            this.correspondingRedAprilTag = correspondingRedAprilTag;
        }

        public Pose2d getCorrespondingAprilTagPose() {
            return Robot.isBlue() ? this.correspondingBlueAprilTag.getLocation().toPose2d() : this.correspondingRedAprilTag.getLocation().toPose2d();
        }

        public Pose2d getBranchPoseProjectedOntoReefFace() {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(0, Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftBranchRobotRelative() ? -1 : 1), Rotation2d.kZero));
        }

        public Pose2d getScorePose(int level, boolean isScoringFrontSide) {
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
                    throw new IllegalArgumentException("Branch level provided to CoralBranch.getScorePose() was invalid. Should be in range [2,4]");
            }

            return getCorrespondingAprilTagPose().transformBy(
                new Transform2d(
                    Constants.LENGTH_WITH_BUMPERS_METERS/2 + targetDistanceFromReef, 
                    Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftBranchRobotRelative() ? -1 : 1) + Constants.SHOOTER_Y_OFFSET * (isScoringFrontSide ? 1 : -1) + (isScoringFrontSide ? 0 : 0.055), 
                    isScoringFrontSide ? Rotation2d.k180deg : Rotation2d.kZero));
        }

        public Pose2d getClearancePose(boolean isScoringFrontSide) {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS/2 + Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM,
                Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftBranchRobotRelative() ? -1 : 1) + Constants.SHOOTER_Y_OFFSET * (isScoringFrontSide ? 1 : -1) + (isScoringFrontSide ? 0 : 0.055), 
                isScoringFrontSide ? Rotation2d.k180deg : Rotation2d.kZero));
        }

        public boolean isLeftBranchRobotRelative() {
            return switch (this) {
                case A, C, E, G, I, K -> true;
                default -> false;
            };
        }

        public boolean isLeftBranchFieldRelative() {
            return switch (this) {
                case A, C, F, H, J, K -> true;
                default -> false;
            };
        }
    }

    public static CoralBranch getClosestCoralBranch() {
        CoralBranch nearestBranch = CoralBranch.A;
        double closestDistance = Double.MAX_VALUE;

        for (CoralBranch branch : CoralBranch.values()) {
            double distance = CommandSwerveDrivetrain.getInstance().getPose().minus(branch.getBranchPoseProjectedOntoReefFace()).getTranslation().getNorm();
            if (distance < closestDistance) {
                closestDistance = distance;
                nearestBranch = branch;
            }
        }

        return nearestBranch;
    }

    public enum ReefFace {
        AB(NamedTags.BLUE_AB, NamedTags.RED_AB, CoralBranch.A, CoralBranch.B, getLineStart(NamedTags.BLUE_AB), getLineEnd(NamedTags.BLUE_AB)),
                CD(NamedTags.BLUE_CD, NamedTags.RED_CD, CoralBranch.C, CoralBranch.D, getLineStart(NamedTags.BLUE_CD), getLineEnd(NamedTags.BLUE_CD)),
                EF(NamedTags.BLUE_EF, NamedTags.RED_EF, CoralBranch.F, CoralBranch.E, getLineStart(NamedTags.BLUE_EF), getLineEnd(NamedTags.BLUE_EF)),
                GH(NamedTags.BLUE_GH, NamedTags.RED_GH, CoralBranch.H, CoralBranch.G, getLineStart(NamedTags.BLUE_GH), getLineEnd(NamedTags.BLUE_GH)),
                IJ(NamedTags.BLUE_IJ, NamedTags.RED_IJ, CoralBranch.J, CoralBranch.I, getLineStart(NamedTags.BLUE_IJ), getLineEnd(NamedTags.BLUE_IJ)),
                KL(NamedTags.BLUE_KL, NamedTags.RED_KL, CoralBranch.K, CoralBranch.L, getLineStart(NamedTags.BLUE_KL), getLineEnd(NamedTags.BLUE_KL));
        
                private NamedTags correspondingBlueAprilTag;
                private NamedTags correspondingRedAprilTag;
                private CoralBranch leftBranchFieldRelative;
                private CoralBranch rightBranchFieldRelative;
                private Translation2d blueOriginLineStart;
                private Translation2d blueOriginLineEnd;
        
                private ReefFace(NamedTags correspondingBlueAprilTag, NamedTags correspondingRedAprilTag, CoralBranch leftBranchFieldRelative, CoralBranch rightBranchFieldRelative, Translation2d blueOriginLineStart, Translation2d blueOriginLineEnd) {
                    this.correspondingBlueAprilTag = correspondingBlueAprilTag;
                    this.correspondingRedAprilTag = correspondingRedAprilTag;
                    this.leftBranchFieldRelative = leftBranchFieldRelative;
                    this.rightBranchFieldRelative = rightBranchFieldRelative;
                    this.blueOriginLineStart = blueOriginLineStart;
                    this.blueOriginLineEnd = blueOriginLineEnd;
                }
        
                public CoralBranch getLeftBranchFieldRelative() {
                    return this.leftBranchFieldRelative;
                }
        
                public CoralBranch getRightBranchFieldRelative() {
                    return this.rightBranchFieldRelative;
                }
        
                public Pose2d getCorrespondingAprilTagPose() {
                    return Robot.isBlue() ? this.correspondingBlueAprilTag.getLocation().toPose2d() : this.correspondingRedAprilTag.getLocation().toPose2d();
                }
        
                public static ReefFace getClosestReefFace() {
                    ReefFace nearestReefFace = ReefFace.AB;
                    double closestDistance = Double.MAX_VALUE;
        
                    for (ReefFace reefFace : ReefFace.values()) {
                        double distance = CommandSwerveDrivetrain.getInstance().getPose().minus(reefFace.getCorrespondingAprilTagPose()).getTranslation().getNorm();
                        if (distance < closestDistance) {
                            closestDistance = distance;
                            nearestReefFace = reefFace;
                        }
                    }
                    return nearestReefFace;
                }
        
                public static NamedTags getNearestReefSide() {
                    return Robot.isBlue() ? getClosestReefFace().correspondingBlueAprilTag : getClosestReefFace().correspondingRedAprilTag;
                }
        
                private static Translation2d getLineStart(NamedTags tag) {
                    Translation2d lineStart = tag.getLocation().toPose2d().transformBy(new Transform2d(Units.inchesToMeters(0), Units.inchesToMeters(-37.04/2), new Rotation2d())).getTranslation();
                    return Robot.isBlue() ? 
                        lineStart :
                        Field.transformToOppositeAlliance(new Pose2d(lineStart, Rotation2d.kZero)).getTranslation();
                }

                private static Translation2d getLineEnd(NamedTags tag) {
                    Translation2d lineEnd = tag.getLocation().toPose2d().transformBy(new Transform2d(Units.inchesToMeters(0), Units.inchesToMeters(37.04/2), new Rotation2d())).getTranslation();
                    return Robot.isBlue() ? 
                        lineEnd :
                        Field.transformToOppositeAlliance(new Pose2d(lineEnd, Rotation2d.kZero)).getTranslation();
                }

                public static Pose2d getTargetPose(NamedTags tag) {
                    Translation2d lineStart = getLineStart(tag);
                    Translation2d lineEnd = getLineEnd(tag);
                    Translation2d robot = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();

                    Vector2D lineStartToEnd = new Vector2D(lineEnd.getX() - lineStart.getX(), lineEnd.getY() - lineStart.getY());
                    Vector2D lineStartToPoint = new Vector2D(robot.getX() - lineStart.getX(), robot.getY() - lineStart.getY());
                    
                    double lineLengthSquared = lineStartToEnd.dot(lineStartToEnd);
                    double dotProduct = lineStartToEnd.dot(lineStartToPoint);
                    
                    double t = dotProduct / lineLengthSquared; // Projection factor
                    
                    Translation2d closestPointOnCoralStation = new Translation2d(lineStart.getX() + t * lineStartToEnd.x, lineStart.getY() + t * lineStartToEnd.y);

                    return new Pose2d(closestPointOnCoralStation, tag.getLocation().toPose2d().getRotation());
                }
    }

    /*** REEF ALGAE ***/

    public enum Algae {
        AB_BLUE(NamedTags.BLUE_AB.getLocation().toPose2d()),
        CD_BLUE(NamedTags.BLUE_CD.getLocation().toPose2d()),
        EF_BLUE(NamedTags.BLUE_EF.getLocation().toPose2d()),
        GH_BLUE(NamedTags.BLUE_GH.getLocation().toPose2d()),
        IJ_BLUE(NamedTags.BLUE_IJ.getLocation().toPose2d()),
        KL_BLUE(NamedTags.BLUE_KL.getLocation().toPose2d()),
        AB_RED(NamedTags.RED_AB.getLocation().toPose2d()),
        CD_RED(NamedTags.RED_CD.getLocation().toPose2d()),
        EF_RED(NamedTags.RED_EF.getLocation().toPose2d()),
        GH_RED(NamedTags.RED_GH.getLocation().toPose2d()),
        IJ_RED(NamedTags.RED_IJ.getLocation().toPose2d()),
        KL_RED(NamedTags.RED_KL.getLocation().toPose2d());

        private Pose2d correspondingTagPose;

        private Algae(Pose2d correspondingTagPose) {
            this.correspondingTagPose = correspondingTagPose;
        }

        public Pose2d getCorrespondingAprilTagPose() {
            return this.correspondingTagPose;
        }

        public boolean isHighAlgae() {
            return switch (this) {
                case AB_BLUE, AB_RED, EF_BLUE, EF_RED, IJ_BLUE, IJ_RED-> true;
                default -> false;
            };
        }

        public Pose2d getTargetPose(boolean isFrontFacingReef) {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS / 2 + (isHighAlgae() ? Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_ALGAE_L3 : Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_ALGAE_L2), 
                isFrontFacingReef ? Constants.SHOOTER_Y_OFFSET : -Constants.SHOOTER_Y_OFFSET, 
                isFrontFacingReef ? Rotation2d.k180deg :  Rotation2d.kZero));
        }

        public Pose2d getReadyPose(boolean isFrontFacingReef) {
            return getTargetPose(isFrontFacingReef).transformBy(new Transform2d(
                isFrontFacingReef ? -0.15 : 0.15,
                0,
                Rotation2d.kZero));
        }
    }

    public static Algae getClosestAlgae() {
        Algae nearestAlgaeBranch = Algae.AB_BLUE;
        double closestDistance = Double.MAX_VALUE;

        for (Algae algaeBranch : Algae.values()) {
            double distance = CommandSwerveDrivetrain.getInstance().getPose().minus(algaeBranch.getTargetPose(true)).getTranslation().getNorm();
            if (distance < closestDistance) {
                closestDistance = distance;
                nearestAlgaeBranch = algaeBranch;
            }
        }

        return nearestAlgaeBranch;
    }
}
