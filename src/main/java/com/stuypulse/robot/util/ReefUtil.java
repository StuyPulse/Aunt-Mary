package com.stuypulse.robot.util;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.NamedTags;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

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
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(0, Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftPeg() ? -1 : 1), Rotation2d.kZero));
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
                    Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftPeg() ? -1 : 1) + Constants.SHOOTER_Y_OFFSET * (isScoringFrontSide ? 1 : -1) + (isScoringFrontSide ? 0 : 0.055), 
                    isScoringFrontSide ? Rotation2d.k180deg : Rotation2d.kZero));
        }

        public Pose2d getClearancePose(boolean isScoringFrontSide) {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS/2 + Settings.Clearances.CLEARANCE_DISTANCE_FROM_REEF_ARM,
                Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftPeg() ? -1 : 1) + Constants.SHOOTER_Y_OFFSET * (isScoringFrontSide ? 1 : -1) + (isScoringFrontSide ? 0 : 0.055), 
                isScoringFrontSide ? Rotation2d.k180deg : Rotation2d.kZero));
        }

        public boolean isLeftPeg() {
            return switch (this) {
                case A, C, E, G, I, K -> true;
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

    /*** REEF FACE ***/
    public enum ReefFace {
        AB(NamedTags.BLUE_AB, NamedTags.RED_AB, CoralBranch.A, CoralBranch.B),
        CD(NamedTags.BLUE_CD, NamedTags.RED_CD, CoralBranch.C, CoralBranch.D),
        EF(NamedTags.BLUE_EF, NamedTags.RED_EF, CoralBranch.F, CoralBranch.E),
        GH(NamedTags.BLUE_GH, NamedTags.RED_GH, CoralBranch.H, CoralBranch.G),
        IJ(NamedTags.BLUE_IJ, NamedTags.RED_IJ, CoralBranch.J, CoralBranch.I),
        KL(NamedTags.BLUE_KL, NamedTags.RED_KL, CoralBranch.K, CoralBranch.L);

        private NamedTags correspondingBlueAprilTag;
        private NamedTags correspondingRedAprilTag;
        private CoralBranch leftBranchFieldRelative;
        private CoralBranch rightBranchFieldRelative;

        private ReefFace(NamedTags correspondingBlueAprilTag, NamedTags correspondingRedAprilTag, CoralBranch leftBranchFieldRelative, CoralBranch rightBranchFieldRelative) {
            this.correspondingBlueAprilTag = correspondingBlueAprilTag;
            this.correspondingRedAprilTag = correspondingRedAprilTag;
            this.leftBranchFieldRelative = leftBranchFieldRelative;
            this.rightBranchFieldRelative = rightBranchFieldRelative;
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

        public boolean isOnDriverStationSide() {
            return switch (this) {
                case KL, AB, CD -> true;
                default -> false;
            };
        }

        public static NamedTags getNearestReefSide() {
            return Robot.isBlue() ? getClosestReefFace().correspondingBlueAprilTag : getClosestReefFace().correspondingRedAprilTag;
        }

        private Translation2d getLineStart() {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(0, Field.LENGTH_OF_REEF_FACE / 2, Rotation2d.kZero)).getTranslation();
        }

        private Translation2d getLineEnd() {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(0, -Field.LENGTH_OF_REEF_FACE / 2, Rotation2d.kZero)).getTranslation();
        }

        public Pose2d getL1FroggyScorePose() {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS / 2 + Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L1_FROGGY, 
                0, 
                Rotation2d.kCW_90deg));
        }

        public Pose2d getL1FroggyClearPose() {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS / 2 + Settings.Clearances.CLEARANCE_DISTANCE_FROGGY + 0.05, 
                0, 
                Rotation2d.kCW_90deg));
        }
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

        public Pose2d getTargetPose() {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS / 2 + (isHighAlgae() ? Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_ALGAE_L3 : Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_ALGAE_L2), 
                Constants.SHOOTER_Y_OFFSET, 
                Rotation2d.k180deg));
        }

        public Pose2d getReadyPose() {
            return getTargetPose().transformBy(new Transform2d(-0.2, 0, Rotation2d.kZero));
        }
    }

    public static Algae getClosestAlgae() {
        Algae nearestAlgaeBranch = Algae.AB_BLUE;
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
}
