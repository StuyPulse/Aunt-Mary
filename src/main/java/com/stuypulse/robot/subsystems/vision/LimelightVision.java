
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.vision;


import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase{

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

    private MegaTagMode megaTagMode;
    private int maxTagCount;

    private LimelightVision() {
        for (Camera camera : Cameras.LimelightCameras) {
            Pose3d robotRelativePose = camera.getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                camera.getName(), 
                robotRelativePose.getX(), 
                -robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getX()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getY()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getZ())
            );
        }

        maxTagCount = 0;

        megaTagMode = MegaTagMode.MEGATAG1;
        CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(Settings.Vision.MT1_STDEVS);

        setIMUMode(1);
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
    }

    public void setTagWhitelist(int... ids) {
        for (Camera camera : Cameras.LimelightCameras) {
            LimelightHelpers.SetFiducialIDFiltersOverride(camera.getName(), ids);
        }
    }

    public void setIMUMode(int mode) {
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
        LimelightHelpers.SetRobotOrientation(
            limelightName, 
            (CommandSwerveDrivetrain.getInstance().getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
            0, 
            0, 
            0, 
            0, 
            0
        );
        return Robot.isBlue() 
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    }

    @Override
    public void periodic() {
        this.maxTagCount = 0;

        for (Camera camera : Cameras.LimelightCameras) {
            if (camera.isEnabled()) {
                PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                    ? getMegaTag2PoseEstimate(camera.getName())
                    : getMegaTag1PoseEstimate(camera.getName());

                Vector<N3> stdevs = (megaTagMode == MegaTagMode.MEGATAG2)
                    ? Settings.Vision.MT2_STDEVS
                    : Settings.Vision.MT1_STDEVS;
                
                if (poseEstimate != null && poseEstimate.tagCount > 0) {
                    CommandSwerveDrivetrain.getInstance().addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdevs);
                    SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", true);
                    SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", poseEstimate.tagCount);
                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                }
                else {
                    SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", false);
                    SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", 0);
                }
            }
        }

        SmartDashboard.putString("Vision/Megatag Mode", this.megaTagMode.toString());
    }
}