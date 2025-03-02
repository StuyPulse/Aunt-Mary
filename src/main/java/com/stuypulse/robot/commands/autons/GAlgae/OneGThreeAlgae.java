// package com.stuypulse.robot.commands.autons.GAlgae;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
// import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL2;
// import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL3;
// import com.stuypulse.robot.commands.arm.algae.ArmToCatapultReady;
// import com.stuypulse.robot.commands.arm.algae.ArmToCatapultShoot;
// import com.stuypulse.robot.commands.arm.algae.ArmWaitUntilCanCatapult;
// import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
// import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
// import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL2;
// import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL3;
// import com.stuypulse.robot.commands.elevator.algae.ElevatorToBarge;
// import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
// import com.stuypulse.robot.commands.leds.LEDApplyPattern;
// import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
// import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
// import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
// import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
// import com.stuypulse.robot.commands.shooter.ShooterStop;
// import com.stuypulse.robot.commands.swerve.SwerveDriveNudgeForward;
// import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
// import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
// import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
// import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
// import com.stuypulse.robot.constants.Settings;
// import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
// import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
// import com.stuypulse.robot.subsystems.shooter.Shooter;
// import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import com.stuypulse.robot.util.ReefUtil.CoralBranch;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// public class OneGThreeAlgae extends SequentialCommandGroup {
    
//     public OneGThreeAlgae(PathPlannerPath... paths) {

//         addCommands(

//             // Score Preload on G
//             new ParallelCommandGroup(
//                 new SwerveDriveCoralScoreAlignWithClearance(CoralBranch.G, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT),
//                 new ElevatorToL4Front().alongWith(new ArmToL4Front())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ShooterShootBackwards(),
//             new WaitCommand(0.2),
//             new ShooterStop(),

//             // Acquire GH Algae, Score on Barge
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
//                 new WaitUntilCommand(() -> CommandSwerveDrivetrain.getInstance().isClearFromReef())
//                     .andThen(
//                         new ElevatorToAlgaeL2().alongWith(new ArmToAlgaeL2())
//                             .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())))
//             ),
                
//             new ShooterAcquireAlgae(),
//             new ShooterHoldAlgae(),
//             new ParallelCommandGroup(
//                 new WaitUntilCommand(() -> )
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
//                 new ElevatorToBarge().alongWith(new ArmToCatapultReady())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ArmToCatapultShoot()
//                 .andThen(new ArmWaitUntilCanCatapult())
//                     .andThen(new ShooterShootAlgae()),
            
//             // Acquire IJ Algae, Score on Barge
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
//                 new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ParallelCommandGroup(
//                 new SwerveDrivePidToNearestReefAlgae()
//                     .alongWith(new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3()))
//                         .alongWith(new ShooterAcquireAlgae())
//                             .andThen(new SwerveDriveNudgeForward())
//             ),
//             new ShooterHoldAlgae(),
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
//                 new ElevatorToBarge().alongWith(new ArmToCatapultReady())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ArmToCatapultShoot()
//                 .andThen(new ArmWaitUntilCanCatapult())
//                     .andThen(new ShooterShootAlgae()),

//             // Acquire EF Algae, Score on Barge
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
//                 new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ParallelCommandGroup(
//                 new SwerveDrivePidToNearestReefAlgae()
//                     .alongWith(new ElevatorToAlgaeL3().alongWith(new ArmToAlgaeL3()))
//                         .alongWith(new ShooterAcquireAlgae())
//                             .andThen(new SwerveDriveNudgeForward())
//             ),
//             new ShooterHoldAlgae(),
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
//                 new ElevatorToBarge().alongWith(new ArmToCatapultReady())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ArmToCatapultShoot()
//                 .andThen(new ArmWaitUntilCanCatapult())
//                     .andThen(new ShooterShootAlgae())
//         );

//     }

// }
