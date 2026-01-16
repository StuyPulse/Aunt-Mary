/** ********************** PROJECT MARY ************************ */
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/** ************************************************************ */
package com.stuypulse.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.Queue;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.AprilTag;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.robot.util.vision.LimelightHelpers.RawDetection;
import com.stuypulse.robot.util.vision.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public enum PipelineMode {
        APRILTAG,
        GAMEPIECE
    }

    private MegaTagMode megaTagMode;

    private int imuMode;
    private int maxTagCount;
    private RawDetection[] rawDetections;
    private RawFiducial[] rawFiducials;

    private boolean[] whitelist = new boolean[Field.APRILTAGS.length];

    private ObjectData currentFrame;
    private ObjectData closestObject;

    private Timer timer;
    private Queue<ServoObjectData> objectFIFO;
    private ServoObjectData lastGood;

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

    public ServoObjectData getLastGood() {
        return lastGood;
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

    private int[] getWhitelist() {
        int[] ids = new int[whitelist.length];
        int num = 0;
        for (int i = 0; i < whitelist.length; i++) {
            if (whitelist[i]) {
                ids[num] = i;
                num++;
            }
        }
        return ids;
    }

    private void setAllWhitelist() {
        for (boolean x : whitelist) {
            x = true;
        }
    }

    private void updateWhitelist() {
        if (Robot.getMode() == RobotMode.DISABLED) {
            if (Robot.isBlue()) {
                whitelist[0] = true;
            }
        }

        // if (Robot.getMode() == RobotMode.DISABLED) { // whitelist alliance tags during disabled loop
        //     if (Robot.isBlue()) {
        //         setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS);
        //     } else {
        //         setWhitelistMode(WhitelistMode.RED_REEF_TAGS);
        //     }
        // } else {
        //     if (robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.RED_REEF_TAGS)) {
        //         setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS);
        //     }
        //     if (!robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.BLUE_REEF_TAGS)) {
        //         setWhitelistMode(WhitelistMode.RED_REEF_TAGS);
        //     }
        //     if (robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS,
        //             WhitelistMode.BLUE_CS_TAGS)) {
        //         setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS, WhitelistMode.BLUE_CS_TAGS);
        //     }
        //     if (robotIsOnBlueSide() && isWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS,
        //             WhitelistMode.RED_CS_TAGS)) {
        //         setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS, WhitelistMode.RED_REEF_TAGS, WhitelistMode.RED_CS_TAGS);
        //     }
        // }
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

    public ServoObjectData getLastServoObject() {
        return objectFIFO.poll();
    }

    @Override
    public void periodic() {
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        this.maxTagCount = 0;

        setAllWhitelist();

        rawFiducials = LimelightHelpers.getRawFiducials("limelight-froggy");
        for (RawFiducial tag : rawFiducials) {
            if (tag.distToRobot > 10.0) {
                whitelist[tag.id] = false;
            }
        }

        // updateWhitelist();
        setTagWhitelist(getWhitelist());

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
                rawDetections = LimelightHelpers.getRawDetections("limelight-froggy");
                rawFiducials = LimelightHelpers.getRawFiducials("limelight-froggy");
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
                    RawDetection[] RawResults = LimelightHelpers.getRawDetections(camera.getName());

                    double closestDistance = Double.MAX_VALUE;

                    while (objectFIFO.size() >= 20) {
                        ServoObjectData temp = objectFIFO.poll();
                        if (temp != null) {
                            lastGood = temp;
                            SmartDashboard.putNumber("Vision/LAST GOOD ANGLE", temp.getObjectAngle());
                            SmartDashboard.putNumber("Vision/LAST GOOD TIME", temp.getTimeStamp());
                        }
                        
                    }

                    for (RawDetection detection : RawResults) {
                        double totalAngleX = Cameras.LimelightCameras[2].getLocation().getRotation().getZ()
                                - Units.degreesToRadians(detection.txnc);
                        double totalAngleY = Cameras.LimelightCameras[2].getLocation().getRotation().getY()
                                + Units.degreesToRadians(detection.tync);

                        ServoObjectData data = new ServoObjectData(totalAngleX, timer.getTimestamp());
                        SmartDashboard.putNumber("Vision Total Angle X", totalAngleX);
                    
                        objectFIFO.add(data);
                          SmartDashboard.putNumber("Vision/FIFO Length", objectFIFO.size());

                    }
                }
            }

            SmartDashboard.putString("Vision/Megatag Mode", getMTmode().toString());
            SmartDashboard.putNumber("Raw Detection length", rawDetections.length);
            SmartDashboard.putBoolean("Vision/Has NN Data", hasNeuralNetworkData("froggy-limelight"));
            SmartDashboard.putNumber("Vision/IMU Mode", imuMode);
        }
    }
}
