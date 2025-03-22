
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public Camera[] LimelightCameras = new Camera[] {
        new Camera("limelight-funnel", new Pose3d(Units.inchesToMeters(-14.057630), Units.inchesToMeters(2.250), Units.inchesToMeters(6.172275), new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(20), Units.degreesToRadians(180))), Settings.EnabledSubsystems.FUNNEL_LIMELIGHT),
        new Camera("limelight-shooter", new Pose3d(Units.inchesToMeters(9.0), Units.inchesToMeters(9.226), Units.inchesToMeters(10.0), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-13.1), Units.degreesToRadians(-18.65))), Settings.EnabledSubsystems.SHOOTER_LIMELIGHT)
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private SmartBoolean isEnabled;

        public Camera(String name, Pose3d location, SmartBoolean isEnabled) {
            this.name = name;
            this.location = location;
            this.isEnabled = isEnabled;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }

        public boolean isEnabled() {
            return isEnabled.get();
        }

        public void setEnabled(boolean enabled) {
            this.isEnabled.set(enabled);
        }
    }
}