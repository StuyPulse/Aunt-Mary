/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.vision.VisionSetMegaTag1;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag2;
import com.stuypulse.robot.commands.vision.VisionSetWhiteList;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    public enum RobotMode {
        DISABLED,
        AUTON,
        TELEOP,
        TEST
    }

    private static RobotMode mode;

    private RobotContainer robot;
    private Command auto;

    public static boolean isBlue() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static RobotMode getMode() {
        return mode;
    }

    /*************************/
    /*** ROBOT SCHEDULING ***/
    /*************************/

    @Override
    public void robotInit() {
        robot = new RobotContainer();
        mode = RobotMode.DISABLED;

        DataLogManager.start();

        // Allows us to see the limelight feeds even while tethered through USB-B 
        for (int port = 5800; port <= 5809; port++){   
            PortForwarder.add(port, "10.6.94.11", port);
            PortForwarder.add(port+10, "10.6.94.12", port);
        }
       
        // Ignore barge tags, processor tags, and coral station tags
        new VisionSetWhiteList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22).schedule();

        // PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        mode = RobotMode.DISABLED;

        if (!DriverStation.isFMSAttached()) {
            new VisionSetMegaTag1().schedule();
        }
    }

    @Override
    public void disabledPeriodic() {}

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/

    @Override
    public void autonomousInit() {
        mode = RobotMode.AUTON;
        auto = robot.getAutonomousCommand();

        new VisionSetMegaTag2().schedule();
        
        if (auto != null) {
            auto.schedule();
        }

        Shuffleboard.selectTab("Autonomous");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        mode = RobotMode.TELEOP;
        if (auto != null) {
            auto.cancel();
        }
        new VisionSetMegaTag2().schedule();

        Shuffleboard.selectTab("Teleoperated");
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        mode = RobotMode.TEST;
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
