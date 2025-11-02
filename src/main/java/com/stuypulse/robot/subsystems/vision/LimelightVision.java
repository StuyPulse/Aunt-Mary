/** ********************** PROJECT MARY ************************ */
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/** ************************************************************ */
package com.stuypulse.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.robot.util.vision.LimelightHelpers.RawDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase {
    private static final LimelightVision instance;

    static {
        instance = new LimelightVision();
    }

    public static LimelightVision getInstance() {
        return instance;
    }

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2
    }

    public enum WhitelistMode {
        BLUE_REEF_TAGS(Field.BLUE_REEF_TAG_IDS),
        RED_REEF_TAGS(Field.RED_REEF_TAG_IDS),
        BLUE_CS_TAGS(Field.BLUE_CS_TAGS),
        RED_CS_TAGS(Field.RED_CS_TAGS),
        BLUE_PROCESSOR_TAG(Field.BLUE_PROCESSOR),
        RED_PROCESSOR_TAG(Field.RED_PROCESSOR),
        BLUE_BARGE_TAGS(Field.BLUE_SIDE_BARGE_TAGS),
        RED_BARGE_TAGS(Field.RED_SIDE_BARGE_TAGS);

        private int[] ids;

        private WhitelistMode(int... ids) {
            this.ids = ids;
        }

        public int[] getIds() {
            return this.ids;
        }
    }

    public enum PipelineMode {
        APRILTAG,
        GAMEPIECE
    }

    private MegaTagMode megaTagMode;
    private WhitelistMode[] whitelistModes;
    private int imuMode;
    private int maxTagCount;
    private RawDetection[] rawDetections;

    private ObjectData currentFrame;
    private ObjectData closestObject;

    private Timer timer;
    private Queue<ServoObjectData> objectFIFO;

    public LimelightVision() {
        for (Camera camera : Cameras.LimelightCameras) {
            Pose3d robotRelativePose = camera.getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                    camera.getName(),
                    robotRelativePose.getX(),
                    -robotRelativePose.getY(),
                    robotRelativePose.getZ(),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getX()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getY()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getZ()));
        }

        maxTagCount = 0;

        setMegaTagMode(MegaTagMode.MEGATAG1);
        setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS);
        setIMUMode(1);

        // Auto Acquire
        currentFrame = new ObjectData(Pose2d.kZero, 0);
        closestObject = new ObjectData(Pose2d.kZero, 0);

        timer = new Timer();
        objectFIFO = new LinkedList<>();
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
        switch (mode) {
            case MEGATAG1:
                CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(Settings.Vision.MT1_STDEVS);
                break;
            case MEGATAG2:
                CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(Settings.Vision.MT2_STDEVS);
                break;
        }
    }

    public void setPipelineMode(int pipeline, String limelightName) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    public void setWhitelistMode(WhitelistMode... modes) {
        int totalLength = 0;

        for (WhitelistMode mode : modes) {
            totalLength += mode.getIds().length;
        }

        int[] combined = new int[totalLength];
        int index = 0;
        for (WhitelistMode mode : modes) {
            for (int id : mode.getIds()) {
                combined[index++] = id;
            }
        }

        setTagWhitelist(combined);
    }

    public WhitelistMode[] getWhitelistModes() {
        return this.whitelistModes;
    }

    public boolean isWhitelistMode(WhitelistMode mode) {
        if (whitelistModes != null) {
            for (WhitelistMode m : whitelistModes) {
                if (m.equals(mode)) {
                    return true;
                }
            }
        }
        return false;
    }

    public boolean isWhitelistMode(WhitelistMode... modes) {
        if (whitelistModes != null) {
            int count = 0;
            for (WhitelistMode mode : modes) {
                for (WhitelistMode m : whitelistModes) {
                    if (m.equals(mode)) {
                        count++;
                    }
                }
            }
            return count == modes.length;
        }
        return false;
    }

    private void setTagWhitelist(int... ids) {
        for (Camera camera : Cameras.LimelightCameras) {
            LimelightHelpers.SetFiducialIDFiltersOverride(camera.getName(), ids);
        }
    }

    public void setIMUMode(int mode) {
        this.imuMode = mode;
        for (Camera camera : Cameras.LimelightCameras) {
            LimelightHelpers.SetIMUMode(camera.getName(), mode);
        }
    }

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    public MegaTagMode getMTmode() {
        return megaTagMode;
    }

    public PoseEstimate getMegaTag1PoseEstimate(String limelightName) {
        return Robot.isBlue()
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
    }

    private PoseEstimate getMegaTag2PoseEstimate(String limelightName) {
        return Robot.isBlue()
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    }

    public ObjectData getObjectFromCurrentFrame() {
        return currentFrame;
    }

    public ObjectData getClosestObject() {
        return closestObject;
    }

    private boolean robotIsOnBlueSide() {
        Pose2d pose = CommandSwerveDrivetrain.getInstance().getPose();
        return pose.getX() < Field.LENGTH / 2 == Robot.isBlue();
    }

    private void updateWhitelistMode() {
        if (Robot.getMode() == RobotMode.DISABLED) { // whitelist alliance tags during disabled loop
            if (Robot.isBlue()) {
                setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS);
            } else {
                setWhitelistMode(WhitelistMode.RED_REEF_TAGS);
            }
        } else {
            if (robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.RED_REEF_TAGS)) {
                setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS);
            }
            if (!robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.BLUE_REEF_TAGS)) {
                setWhitelistMode(WhitelistMode.RED_REEF_TAGS);
            }
            if (robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS,
                    WhitelistMode.BLUE_CS_TAGS)) {
                setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS, WhitelistMode.BLUE_CS_TAGS);
            }
            if (robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS,
                    WhitelistMode.RED_CS_TAGS)) {
                setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS, WhitelistMode.RED_CS_TAGS);
            }
        }
    }

    // public Supplier<RawDetection[]> getLimelightRawDetections(String
    // limelightName) {
    // return () -> LimelightHelpers.getRawDetections(limelightName);
    // }
    public Rotation2d getHorizontalTargetAngle(String limelightName) {
        if (hasNeuralNetworkData(limelightName)) {
            return new Rotation2d(rawDetections[0].txnc);
        }
        return null;
    }

    public boolean hasNeuralNetworkData(String limelightName) {
        return rawDetections.length > 0;
    }

    public Queue<ServoObjectData> getNeuralNetworkFIFO() {
        return objectFIFO;
    }

    @Override
    public void periodic() {
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        this.maxTagCount = 0;

        updateWhitelistMode();

        for (Camera camera : Cameras.LimelightCameras) {
            LimelightHelpers.SetRobotOrientation(
                    camera.getName(),
                    (CommandSwerveDrivetrain.getInstance().getPose().getRotation().getDegrees()
                            + (Robot.isBlue() ? 0 : 180)) % 360,
                    0,
                    0,
                    0,
                    0,
                    0);

            if (camera.isEnabled()) {
                rawDetections = LimelightHelpers.getRawDetections("froggy-limelight");
                if (LimelightHelpers.getCurrentPipelineIndex(camera.getName()) == PipelineMode.APRILTAG.ordinal()) {
                    PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                            ? getMegaTag2PoseEstimate(camera.getName())
                            : getMegaTag1PoseEstimate(camera.getName());

                    if (poseEstimate != null && poseEstimate.tagCount > 0) {
                        CommandSwerveDrivetrain.getInstance().addVisionMeasurement(poseEstimate.pose,
                                poseEstimate.timestampSeconds);
                        SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", true);
                        SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", poseEstimate.tagCount);
                        maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                    } else {
                        SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", false);
                        SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", 0);
                    }
                } else if (LimelightHelpers.getCurrentPipelineIndex(camera.getName()) == PipelineMode.GAMEPIECE
                        .ordinal()) {
                    SmartDashboard.putNumber("Vision/WHATTT", 0);
                    RawDetection[] RawResults = LimelightHelpers.getRawDetections(camera.getName());

                    double closestDistance = Double.MAX_VALUE;

                    while (objectFIFO.size() >= 20) {
                        objectFIFO.poll();
                    }

                    for (RawDetection detection : RawResults) {
                        // Translation2d coralPose =
                        // ObjectData.calculateCoralTranslation(detection.txnc, detection.tync);
                        //
                        double totalAngleX = Cameras.LimelightCameras[2].getLocation().getRotation().getZ()
                                - Units.degreesToRadians(detection.txnc);
                        double totalAngleY = Cameras.LimelightCameras[2].getLocation().getRotation().getY()
                                + Units.degreesToRadians(detection.tync);

                        // SmartDashboard.putNumber("Vision/Coral Translation R Camera X",
                        // coralPose.getX());
                        // SmartDashboard.putNumber("Vision/Coral Translation R Camera Y",
                        // coralPose.getY());
                        // coralPose = coralPose.plus(camera.getLocation().toPose2d().getTranslation());
                        // // turn the coral pose relative to the center of robot
                        // as opposed to the camera!
                        // Pose2d fieldCoralPose = robotPose.transformBy(new Transform2d(coralPose, new
                        // Rotation2d()));
                        // double distance =
                        // robotPose.getTranslation().getDistance(fieldCoralPose.getTranslation());

                        // ObjectData data = new ObjectData(fieldCoralPose, timer.get());

                        // if (distance < closestDistance) { // meters
                        // closestDistance = distance;
                        // closestObject = data
                        // }

                        ServoObjectData data = new ServoObjectData(totalAngleX, timer.getTimestamp());
                        objectFIFO.add(data);

                        SmartDashboard.putNumber("Vision/Coral X Pose Meters", currentFrame.objectPose.getX());
                        SmartDashboard.putNumber("Vision/Coral Y Pose Meters", currentFrame.objectPose.getY());

                    }
                }
            }

            SmartDashboard.putString("Vision/Megatag Mode", getMTmode().toString());
            // SmartDashboard.putString("Vision/Whitelist Mode",
            // getWhitelistModes().toString()); // crashes code rn lol
            SmartDashboard.putBoolean("Vision/Has NN Data", hasNeuralNetworkData("froggy-limelight"));
            SmartDashboard.putNumber("Vision/IMU Mode", imuMode);
        }
    }
}
