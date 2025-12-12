/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.stuypulse.robot.commands.shooter.ShooterSetConfigMode;
import com.stuypulse.robot.commands.vision.VisionSetIMUMode;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag1;
import com.stuypulse.robot.commands.vision.VisionSetMegaTag2;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
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

    private static CommandScheduler commandScheduler;
    private static Alliance alliance;
    private static RobotMode mode;

    private RobotContainer robot;
    private Command auto;

    public static boolean isBlue() {
        return alliance == Alliance.Blue;
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
        alliance = Alliance.Blue; //DEFAULT VALUE

        DataLogManager.start();
        SignalLogger.start();

        // Allows us to see the limelight feeds even while tethered through USB-B 
        for (int port = 5800; port <= 5809; port++){   
            PortForwarder.add(port, "10.6.94.11", port);
            PortForwarder.add(port+10, "10.6.94.12", port);
        }

        PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {       
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        commandScheduler = CommandScheduler.getInstance();
        mode = RobotMode.DISABLED;

        // stop using debrecated .schedule method
        if (!DriverStation.isFMSAttached()) {
            //commandScheduler.schedule(new VisionSetMegaTag1());
            commandScheduler.schedule(new VisionSetMegaTag1().andThen(new VisionSetIMUMode(1)));
        }

        commandScheduler.schedule(new ShooterSetConfigMode(NeutralModeValue.Coast));
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
        commandScheduler = CommandScheduler.getInstance();

        commandScheduler.schedule(new VisionSetMegaTag2().andThen(new VisionSetIMUMode(2)));
        //commandScheduler.schedule(new VisionSetMegaTag2());
        commandScheduler.schedule(new ShooterSetConfigMode(NeutralModeValue.Brake) );
        
        if (auto != null) {
            //auto.schedule();
        }

        // Shuffleboard.selectTab("Autonomous");
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

        commandScheduler.schedule(new VisionSetMegaTag2().andThen(new VisionSetIMUMode(2)));
        //commandScheduler.schedule(new VisionSetMegaTag2());
        commandScheduler.schedule(new ShooterSetConfigMode(NeutralModeValue.Brake));
        // Shuffleboard.selectTab("Teleoperated");
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
