package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.CoralBranch;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveFullAlignment extends SequentialCommandGroup {
    private Elevator elevator;
    private Arm arm;
    private CommandSwerveDrivetrain swerve;
    private ElevatorState targetElevatorState;
    private ArmState targetArmState;
    
    public SwerveDriveFullAlignment(int level, boolean isFrontFacingReef) {
        CoralBranch nearestBranch = Field.getClosestBranch();

        switch (level) {
            case 4:
                if (isFrontFacingReef) {
                    targetElevatorState = ElevatorState.L4_FRONT;
                    targetArmState = ArmState.L4_FRONT;
                } else {
                    targetElevatorState = ElevatorState.L4_BACK;
                    targetArmState = ArmState.L4_BACK;
                }
                break;
            case 3:
                if (isFrontFacingReef) {
                    targetElevatorState = ElevatorState.L3_FRONT;
                    targetArmState = ArmState.L3_FRONT;
                } else {
                    targetElevatorState = ElevatorState.L3_BACK;
                    targetArmState = ArmState.L3_BACK;
                }
                break;
            case 2:
                if (isFrontFacingReef) {
                    targetElevatorState = ElevatorState.L2_FRONT;
                    targetArmState = ArmState.L2_FRONT;
                } else {
                    targetElevatorState = ElevatorState.L2_BACK;
                    targetArmState = ArmState.L2_BACK;
                }
                break;
            default:
                    throw new IllegalArgumentException("Branch level provided to SwerveDriveFullAlignment was invalid. Should be in range [2,4]");
        }

        addCommands(
            new SwerveDrivePIDToNearestBranchReady(isFrontFacingReef, nearestBranch)
                .until(() -> elevator.getState() == targetElevatorState && elevator.atTargetHeight() && arm.getState() == targetArmState && arm.atTargetAngle()),
            new SwerveDrivePIDToNearestBranchScore(level, isFrontFacingReef, nearestBranch)
        );

        addRequirements(swerve);
    }
}
