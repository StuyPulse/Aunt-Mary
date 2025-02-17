package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose3d;
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

    private String[] names;
    private SmartBoolean enabled;
    private SmartBoolean[] camerasEnabled;

    public LimelightVision() {
        names = new String[Cameras.LimelightCameras.length];
        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            names[i] = Cameras.LimelightCameras[i].getName();
            Pose3d robotRelativePose = Cameras.LimelightCameras[i].getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                names[i], 
                robotRelativePose.getX(), 
                robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                robotRelativePose.getRotation().getX(), 
                robotRelativePose.getRotation().getY(), 
                robotRelativePose.getRotation().getZ()
            );
        }

        camerasEnabled = new SmartBoolean[Cameras.LimelightCameras.length];
        for (int i = 0; i < camerasEnabled.length; i++) {
            camerasEnabled[i] = new SmartBoolean("Vision/" + names[i] + " Is Enabled", true);
            LimelightHelpers.SetIMUMode(names[i], 0);
            SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
        }

        enabled = new SmartBoolean("Vision/Is Enabled", true);

        CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(Settings.Vision.MIN_STDDEVS);
    }

    public void setTagWhitelist(int... ids) {
        for (String name : names) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        }
    }

    public void enable() {
        enabled.set(true);
    }

    public void disable() {
        enabled.set(false);
    }

    public void setCameraEnabled(String name, boolean enabled) {
        for (int i = 0; i < names.length; i++) {
            if (names[i].equals(name)) {
                camerasEnabled[i].set(enabled);
            }
        }
    }

    public void setIMUMode(int mode) {
        for (String name : names) {
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }

    @Override
    public void periodic() {
        if (enabled.get()) {
            for (int i = 0; i < names.length; i++) {
                if (camerasEnabled[i].get()) {
                    String limelightName = names[i];

                    LimelightHelpers.SetRobotOrientation(
                        limelightName, 
                        (CommandSwerveDrivetrain.getInstance().getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0
                    );

                    PoseEstimate poseEstimate = Robot.isBlue() 
                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
                        : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
                    
                    if (poseEstimate != null && poseEstimate.tagCount > 0) {
                        CommandSwerveDrivetrain.getInstance().addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, Settings.Vision.MIN_STDDEVS.times(1 + poseEstimate.avgTagDist));
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", true);
                    }
                    else {
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
                    }
                }
            }
        }
    }
}
 