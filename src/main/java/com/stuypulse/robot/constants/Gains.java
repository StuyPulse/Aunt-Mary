/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

public interface Gains {

    public interface Elevator {
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
    }

    public interface Arm {
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
    }

    public interface Froggy {
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
    }

    public interface Climb {
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
    }
}
