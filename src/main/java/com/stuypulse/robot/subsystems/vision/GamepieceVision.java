package com.stuypulse.robot.subsystems.vision;

import java.util.ArrayList;

import com.ctre.phoenix.time.StopWatch;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.RawDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamepieceVision extends LimelightVision {

    private static final GamepieceVision instance;
    private ArrayList<Coral> coralList = new ArrayList<Coral>();
    private Pose2d bestCoralPose = new Pose2d();

    static {
        instance = new GamepieceVision();
    }

    public static GamepieceVision getInstance() {
        return instance;
    }

    private StopWatch stopWatch = new StopWatch();

    public GamepieceVision() {
        super();
    }

        class Coral {
            Pose2d coralPose;
            Translation2d coralTranslation2d;
            double detectionTime;

            public Coral(Pose2d coralPose, Translation2d coralTranslation2d, double detectionTime) {
                    this.coralPose = coralPose;
                    this.coralTranslation2d = coralTranslation2d;
                    this.detectionTime = detectionTime;
                }
        }
    
   
    public Translation2d calculateTransformToCoral(double tx, double ty) {
        Pose3d froggyCameraPose3d = Cameras.LimelightCameras[2].getLocation();

        double robotAngleY = Units.radiansToDegrees(froggyCameraPose3d.getRotation().getY());
        double totalAngleY = robotAngleY + ty;
        double xDistance = (froggyCameraPose3d.getZ() - Field.CORAL_RADIUS) / Math.tan(Units.degreesToRadians(totalAngleY));


        double hypotenuseToGround = Math.hypot(xDistance, (froggyCameraPose3d.getZ() - Field.CORAL_RADIUS));

        double yDistance = hypotenuseToGround * Math.tan(tx + froggyCameraPose3d.getY());

        return new Translation2d(xDistance, yDistance);
    }

    public Pose2d getCoralPose() {
        return bestCoralPose;
    }
    
    @Override
    public void periodic() {
        stopWatch.start();

        if (LimelightHelpers.getCurrentPipelineIndex("limelight-froggy") == 1) {
            Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
            RawDetection[] detections = LimelightHelpers.getRawDetections("limelight-froggy");
            Translation2d coralTranslation2d = new Translation2d();
            double now = stopWatch.getDuration();

            coralList.removeIf((Coral) -> now - Coral.detectionTime > 2.0);    

            while (coralList.size() > 10) {
                coralList.remove(0);
            }

            for (RawDetection detection : detections) {
                double tx = detection.txnc;
                double ty = detection.tync;

                coralTranslation2d = calculateTransformToCoral(tx, ty).minus(Cameras.LimelightCameras[2].getLocation().getTranslation().toTranslation2d());

                Pose2d coralPose = robotPose.transformBy(new Transform2d(coralTranslation2d, Cameras.LimelightCameras[2].getLocation().getRotation().toRotation2d()));

                coralList.add(new Coral(coralPose, coralTranslation2d, now));

            }

            for (Coral coral : coralList) {
                if (bestCoralPose == null || bestCoralPose.getTranslation().getDistance(robotPose.getTranslation()) > 
                    coral.coralPose.getTranslation().getDistance(robotPose.getTranslation())) {
                    bestCoralPose = coral.coralPose;
                }
            }
            }
            SmartDashboard.putNumber("Vision/Best Coral X", bestCoralPose.getX());
            SmartDashboard.putNumber("Vision/Best Coral Y", bestCoralPose.getY());
        }
    }