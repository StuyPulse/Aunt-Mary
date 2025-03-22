
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;

/** This class stores information about a tag. */
public class AprilTag {

    private final int id;
    private final Pose3d location;

    public AprilTag(int id, Pose3d location) {
        this.id = id;
        this.location = location;
    }

    public int getID() {
        return id;
    }

    public Pose3d getLocation() {
        return location;
    }
}
