/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public interface Gains {

    public interface Swerve {
        public interface Alignment {
            PIDConstants XY = new PIDConstants(3.5, 0, 0.2);
            PIDConstants THETA = new PIDConstants(3.0, 0, 0.1);
        }
    }

    public interface Elevator {
        public interface PID {
            double kP = 17.185;
            double kI = 0.0;
            double kD = 1.4344;
        }

        public interface FF {
            double kS = 0.28562;
            double kV = 2.8012;
            double kA = 0.16141;
            double kG = 0.81505;
        }
    }

    public interface Arm {
        public interface CoralAlgae  {
            public interface PID {
                double kP = 0.085744;
                double kI = 0.0;
                double kD = 0.015;
            }

            public interface FF {
                double kS = 0.2272; 
                double kV = 0.0095901;
                double kA = 0.0014361;
                double kG = 0.69269;
            }
        }

        public interface Empty  {
            public interface PID {
                double kP = 0.022194;
                double kI = 0.0;
                double kD = 0.00021311;
            }

            public interface FF {
                double kS = 0.041733; 
                double kV = 0.011199;
                double kA = 0.00123655;
                double kG = 0.60473;
            }
        }
    }

    public interface Froggy {
        public interface PID {
            double kP = 0.01571;
            double kI = 0.0;
            double kD = 0.00017229;
        }

        public interface FF {
            double kS = 0.1458; 
            double kV = 0.015181;
            double kA = 0.00059012;
            double kG = 0.24373;
        }
    }
}
