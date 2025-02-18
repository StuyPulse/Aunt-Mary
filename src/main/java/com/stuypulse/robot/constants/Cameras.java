package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public CameraInfo[] LimelightCameras = new CameraInfo[] {
        new CameraInfo("limelight", new Pose3d())
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