package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public CameraInfo[] LimelightCameras = new CameraInfo[] {
        new CameraInfo("funnel", new Pose3d(Units.inchesToMeters(-13.819), Units.inchesToMeters(2.250), Units.inchesToMeters(6.829), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0)))),
        new CameraInfo("shooter", new Pose3d(Units.inchesToMeters(9.0), Units.inchesToMeters(9.226), Units.inchesToMeters(10.0), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-13.1), Units.degreesToRadians(-18.65))))
    };

    public static class CameraInfo {
        private String name;
        private Pose3d location;

        public CameraInfo(String name, Pose3d location) {
            this.name = name;
            this.location = location;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }
    }
}