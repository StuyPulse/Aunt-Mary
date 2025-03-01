// package com.stuypulse.robot.commands.autons.HAlgae;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
// import com.stuypulse.robot.commands.arm.algae.ArmToAlgaeL2;
// import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
// import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
// import com.stuypulse.robot.commands.elevator.algae.ElevatorToAlgaeL2;
// import com.stuypulse.robot.commands.elevator.algae.ElevatorToBarge;
// import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
// import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
// import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
// import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
// import com.stuypulse.robot.commands.shooter.ShooterStop;
// import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
// import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
// import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import com.stuypulse.robot.util.ReefUtil.CoralBranch;
// import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
// import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// public class OneHOneAlgae extends SequentialCommandGroup {
    
//     public OneHOneAlgae(PathPlannerPath... paths) {

//         addCommands(

//             // Score Preload on H
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
//                 new ElevatorToL4Front().alongWith(new ArmToL4Front())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new SwerveDriveCoralScoreAlignWithClearance(CoralBranch.H, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT),
//             new ShooterShootBackwards(),
//             new WaitCommand(3),
//             new ShooterStop(),

//             // Acquire GH Algae, Score on Barge
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
//                 new ElevatorToAlgaeL2().alongWith(new ArmToAlgaeL2())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ShooterAcquireAlgae(),
//             new ParallelCommandGroup(
//                 CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
//                 new ElevatorToBarge().alongWith(new ArmToBarge())
//                     .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
//             ),
//             new ShooterShootAlgae()
//         );

//     }

// }
