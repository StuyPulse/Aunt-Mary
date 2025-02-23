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

public interface ReefUtil {

    public enum CoralBranch {
        A(NamedTags.BLUE_AB.getLocation().toPose2d(), NamedTags.RED_AB.getLocation().toPose2d()),
        B(NamedTags.BLUE_AB.getLocation().toPose2d(), NamedTags.RED_AB.getLocation().toPose2d()),
        C(NamedTags.BLUE_CD.getLocation().toPose2d(), NamedTags.RED_CD.getLocation().toPose2d()),
        D(NamedTags.BLUE_CD.getLocation().toPose2d(), NamedTags.RED_CD.getLocation().toPose2d()),
        E(NamedTags.BLUE_EF.getLocation().toPose2d(), NamedTags.RED_EF.getLocation().toPose2d()),
        F(NamedTags.BLUE_EF.getLocation().toPose2d(), NamedTags.RED_EF.getLocation().toPose2d()),
        G(NamedTags.BLUE_GH.getLocation().toPose2d(), NamedTags.RED_GH.getLocation().toPose2d()),
        H(NamedTags.BLUE_GH.getLocation().toPose2d(), NamedTags.RED_GH.getLocation().toPose2d()),
        I(NamedTags.BLUE_IJ.getLocation().toPose2d(), NamedTags.RED_IJ.getLocation().toPose2d()),
        J(NamedTags.BLUE_IJ.getLocation().toPose2d(), NamedTags.RED_IJ.getLocation().toPose2d()),
        K(NamedTags.BLUE_KL.getLocation().toPose2d(), NamedTags.RED_KL.getLocation().toPose2d()),
        L(NamedTags.BLUE_KL.getLocation().toPose2d(), NamedTags.RED_KL.getLocation().toPose2d());

        private Pose2d correspondingRedAprilTagPose;
        private Pose2d correspondingBlueAprilTagPose;

        private CoralBranch(Pose2d correspondingBlueAprilTagPose, Pose2d correspondingRedAprilTagPose) {
            this.correspondingBlueAprilTagPose = correspondingBlueAprilTagPose;
            this.correspondingRedAprilTagPose = correspondingRedAprilTagPose;
        }

        public Pose2d getCorrespondingAprilTagPose() {
            return Robot.isBlue() ? this.correspondingBlueAprilTagPose : this.correspondingRedAprilTagPose;
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
                    Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftPeg() ? -1 : 1) + Constants.SHOOTER_Y_OFFSET * (isScoringFrontSide ? 1 : -1), 
                    isScoringFrontSide ? Rotation2d.k180deg : Rotation2d.kZero));
        }

        public Pose2d getReadyPose(boolean isScoringFrontSide) {
            return getCorrespondingAprilTagPose().transformBy(new Transform2d(
                Constants.LENGTH_WITH_BUMPERS_METERS/2 + Settings.CLEARANCE_DISTANCE_FROM_REEF + 0.1, // The 0.1 is a buffer to ensure the robot gets past the clearance
                Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftPeg() ? -1 : 1) + Constants.SHOOTER_Y_OFFSET * (isScoringFrontSide ? 1 : -1), 
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

    /*** REEF ALGAE ***/

    public enum Algae {
        AB(NamedTags.BLUE_AB.getLocation().toPose2d(), NamedTags.RED_AB.getLocation().toPose2d()),
        CD(NamedTags.BLUE_CD.getLocation().toPose2d(), NamedTags.RED_CD.getLocation().toPose2d()),
        EF(NamedTags.BLUE_EF.getLocation().toPose2d(), NamedTags.RED_EF.getLocation().toPose2d()),
        GH(NamedTags.BLUE_GH.getLocation().toPose2d(), NamedTags.RED_GH.getLocation().toPose2d()),
        IJ(NamedTags.BLUE_IJ.getLocation().toPose2d(), NamedTags.RED_IJ.getLocation().toPose2d()),
        KL(NamedTags.BLUE_KL.getLocation().toPose2d(), NamedTags.RED_KL.getLocation().toPose2d());

        private Pose2d correspondingRedAprilTagPose;
        private Pose2d correspondingBlueAprilTagPose;

        private Algae(Pose2d correspondingBlueAprilTagPose, Pose2d correspondingRedAprilTagPose) {
            this.correspondingBlueAprilTagPose = correspondingBlueAprilTagPose;
            this.correspondingRedAprilTagPose = correspondingRedAprilTagPose;
        }

        public Pose2d getCorrespondingAprilTagPose() {
            return Robot.isBlue() ? this.correspondingBlueAprilTagPose : this.correspondingRedAprilTagPose;
        }

        public boolean isHighAlgae() {
            return switch (this) {
                case AB, EF, IJ -> true;
                default -> false;
            };
        }

        public Pose2d getTargetPose() {
            return getCorrespondingAprilTagPose()
                .transformBy(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2 + (isHighAlgae() ? Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_ALGAE_L3 : Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_ALGAE_L2), 0, Rotation2d.fromDegrees(180)));
        }
    }

    public static Algae getClosestAlgae() {
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
}
