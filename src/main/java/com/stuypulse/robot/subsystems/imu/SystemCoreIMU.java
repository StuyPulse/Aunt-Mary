package com.stuypulse.robot.subsystems.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.OnboardIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SystemCoreIMU extends SubsystemBase {
   private static final SystemCoreIMU instance;
   private OnboardIMU imu;

    static {
      instance = new SystemCoreIMU();
    }

    public static SystemCoreIMU getInstance() {
        return instance;
    }

    public SystemCoreIMU() {
      imu = new OnboardIMU(OnboardIMU.MountOrientation.kPortrait);
    }

    public Rotation2d getSystemCoreRotation() {
      return imu.getRotation2d();
    }


    @Override
    public void periodic() {
    }
}
