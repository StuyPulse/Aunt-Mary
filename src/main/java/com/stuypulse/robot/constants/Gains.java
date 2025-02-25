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
            PIDConstants XY = new PIDConstants(3.0, 0, 0.0);
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
        public interface Coral  {
            public interface PID {
                double kP = 0.069744;
                double kI = 0.0;
                double kD = 0.015;
            }

            public interface FF {
                double kS = 0.076049; 
                double kV = 0.011872;
                double kA = 0.0011738;
                double kG = 0.69106;
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

        public interface Algae  {
            public interface PID {
                double kP = 0.014094;
                double kI = 0.0;
                double kD = 0.0;
            }

            public interface FF {
                double kS = 0.078914; 
                double kV = 0.011835;
                double kA = 0.0011797;
                double kG = 0.69059;
            }
        }
    }

    public interface Froggy {
        public interface Coral  {
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

        public interface Algae  {
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
