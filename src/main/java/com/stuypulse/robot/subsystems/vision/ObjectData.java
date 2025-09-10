package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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

        double robotAngleY = froggyCameraRotation3d.getY();
        double totalAngleY = robotAngleY + Units.degreesToRadians(tync);

        double coralToRobotHeight = froggyCameraPose3d.getZ()
         - Units.inchesToMeters(Constants.Gamepiece.CORAL_RADIUS);

         SmartDashboard.putNumber("Vision/Height", coralToRobotHeight);
         SmartDashboard.putNumber("Vision/Angle", totalAngleY);

        double xDistance = (coralToRobotHeight) / Math.tan(totalAngleY);

        double hypotenuseToGround = Math.hypot(xDistance, coralToRobotHeight);
        // WHERE X IS FORWARD DISTANCE, Y IS SIDEWAYS DISTANCE
        double yDistance = hypotenuseToGround * Math.tan(Units.degreesToRadians(txnc) + froggyCameraRotation3d.getX());

        return new Translation2d(xDistance, yDistance);
    }
}
