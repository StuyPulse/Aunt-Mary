package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ObjectData {

    Pose2d objectPose;
    double timestamp;

    public ObjectData(Pose2d Pose, double timestamp) {
        this.objectPose = Pose;
        this.timestamp = timestamp;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Pose2d getObjectPose() {
        return objectPose;
    }

    public static Translation2d calculateCoralTranslation(double txnc, double tync) {
        Pose3d froggyCameraPose3d = Cameras.LimelightCameras[2].getLocation();

        Rotation3d froggyCameraRotation3d = froggyCameraPose3d.getRotation();

        double totalAngleX = froggyCameraRotation3d.getZ() - Units.degreesToRadians(txnc); 
        double totalAngleY = froggyCameraRotation3d.getY() + Units.degreesToRadians(tync);

        double totalHeight = froggyCameraPose3d.getZ()
         - Units.inchesToMeters(Constants.Gamepiece.CORAL_DIAMETER);

        SmartDashboard.putNumber("Vision/Total angle X", 180.0 / Math.PI * totalAngleX); // good
        SmartDashboard.putNumber("Vision/Total angle Y", 180.0 / Math.PI * totalAngleY); // good
        SmartDashboard.putNumber("Vision/Height", totalHeight); // good
        SmartDashboard.putNumber("Vision/Angle", totalAngleY); // didnt check

        double xDistance = (totalHeight) * Math.tan(totalAngleY); // bad

        // WHERE X IS FORWARD DISTANCE, Y IS SIDEWAYS DISTANCE
        double yDistance = totalHeight * Math.tan(totalAngleX) / Math.cos(totalAngleY); // bad

        return new Translation2d(xDistance, yDistance);
    }
}
