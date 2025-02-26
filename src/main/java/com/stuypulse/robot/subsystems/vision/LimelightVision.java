package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose3d;
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
    private String[] names;
    private SmartBoolean[] camerasEnabled;
    private int[] tagCounts;

    private LimelightVision() {
        names = new String[Cameras.LimelightCameras.length];
        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            names[i] = Cameras.LimelightCameras[i].getName();
            Pose3d robotRelativePose = Cameras.LimelightCameras[i].getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                names[i], 
                robotRelativePose.getX(), 
                robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getX()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getY()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getZ())
            );
        }

        camerasEnabled = new SmartBoolean[Cameras.LimelightCameras.length];
        tagCounts = new int[Cameras.LimelightCameras.length];

        for (int i = 0; i < camerasEnabled.length; i++) {
            camerasEnabled[i] = new SmartBoolean("Vision/" + names[i] + " Is Enabled", true);
            SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
        }

        megaTagMode = MegaTagMode.MEGATAG1;

        setIMUMode(1);

        CommandSwerveDrivetrain.getInstance().setVisionMeasurementStdDevs(Settings.Vision.MIN_STDDEVS);
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
    }

    public void setTagWhitelist(int... ids) {
        for (String name : names) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        }
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

    public int getMaxTagCount() {
        int maxTagCount = 0;
        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            maxTagCount = Math.max(maxTagCount, tagCounts[i]);
        }
        return maxTagCount;
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
        if (Settings.EnabledSubsystems.VISION.get()) {
            for (int i = 0; i < names.length; i++) {
                if (camerasEnabled[i].get()) {
                    String limelightName = names[i];

                    PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                        ? getMegaTag2PoseEstimate(limelightName)
                        : getMegaTag1PoseEstimate(limelightName);
                    
                    if (poseEstimate != null && poseEstimate.tagCount > 0) {
                        CommandSwerveDrivetrain.getInstance().addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, Settings.Vision.MIN_STDDEVS.times(1 + poseEstimate.avgTagDist));
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", true);
                        SmartDashboard.putNumber("Vision/" + names[i] + " Tag Count", poseEstimate.tagCount);
                        tagCounts[i] = poseEstimate.tagCount;
                    }
                    else {
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
                        SmartDashboard.putNumber("Vision/" + names[i] + " Tag Count", 0);
                        tagCounts[i] = 0;
                    }
                }
            }
        }

        SmartDashboard.putString("Vision/Megatag Mode", this.megaTagMode.toString());
    }
}
 