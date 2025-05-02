
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.superStructure;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL1Back;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL1Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL2Back;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL2Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL3Back;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL3Front;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Back;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{
    private static final SuperStructure instance;

    static {
        instance = new SuperStructure();
    }

    public static SuperStructure getInstance() {
        return instance;
    }

    public enum SuperStructureState {
        FEED(ElevatorState.FEED, ArmState.FEED),
        GOLF_TEE_ALGAE_PICKUP(ElevatorState.GOLF_TEE_ALGAE_PICKUP, ArmState.GOLF_TEE_ALGAE_PICKUP),
        L1_FRONT(ElevatorState.L1_FRONT, ArmState.L1_FRONT),
        L1_BACK(ElevatorState.L1_BACK, ArmState.L1_BACK),
        L2_FRONT(ElevatorState.L2_FRONT, ArmState.L2_FRONT),
        L2_BACK(ElevatorState.L2_BACK, ArmState.L2_BACK),
        L3_FRONT(ElevatorState.L3_FRONT, ArmState.L3_FRONT),
        L3_BACK(ElevatorState.L3_BACK, ArmState.L3_BACK),
        L4_FRONT(ElevatorState.L4_FRONT, ArmState.L4_FRONT),
        L4_BACK(ElevatorState.L4_BACK, ArmState.L4_BACK),
        ALGAE_L2_FRONT(ElevatorState.ALGAE_L2_FRONT, ArmState.ALGAE_L2_FRONT),
        ALGAE_L3_FRONT(ElevatorState.ALGAE_L3_FRONT, ArmState.ALGAE_L3_FRONT),
        ALGAE_L2_BACK(ElevatorState.ALGAE_L2_BACK, ArmState.ALGAE_L2_BACK),
        ALGAE_L3_BACK(ElevatorState.ALGAE_L3_BACK, ArmState.ALGAE_L3_BACK),
        CATAPULT_READY(ElevatorState.CATAPULT, ArmState.CATAPULT_READY),
        CATAPULT_SHOOT(ElevatorState.CATAPULT, ArmState.CATAPULT_SHOOT),
        BARGE_118(ElevatorState.BARGE_118, ArmState.BARGE_118),
        PROCESSOR(ElevatorState.PROCESSOR, ArmState.PROCESSOR),
        CLIMB(ElevatorState.CLIMB, ArmState.CLIMB),
        UNSTUCK_CORAL(ElevatorState.UNSTUCK_CORAL, ArmState.UNSTUCK_CORAL);

        private ElevatorState elevatorState;
        private ArmState armstate;

        private SuperStructureState(ElevatorState elevatorState, ArmState armState) {
            this.elevatorState = elevatorState;
            this.armstate = armState;
        }

        public ElevatorState getElevatorState() {
            return this.elevatorState;
        }

        public ArmState getArmState() {
            return this.armstate;
        }
    }

    private SuperStructureState state;

    private final Arm arm;
    private final Elevator elevator;

    private SuperStructure() {
        this.state = SuperStructureState.FEED;
        this.arm = Arm.getInstance();
        this.elevator = Elevator.getInstance();
    }

    public void setState(SuperStructureState state) {
        this.state = state;
        arm.setState(state.getArmState());
        elevator.setState(state.getElevatorState());
        updateArmMotionProfileConstraints();
        updateElevatorMotionProfileConstraints();
    }

    public SuperStructureState getState() {
        return this.state;
    }

    public boolean isScoringCoral() {
        return state == SuperStructureState.L4_FRONT
            || state == SuperStructureState.L4_BACK
            || state == SuperStructureState.L3_FRONT
            || state == SuperStructureState.L3_BACK
            || state == SuperStructureState.L2_FRONT
            || state == SuperStructureState.L2_BACK
            || state == SuperStructureState.L1_FRONT
            || state == SuperStructureState.L1_BACK;
    }

    public boolean atTarget() {
        return Arm.getInstance().atTargetAngle() && Elevator.getInstance().atTargetHeight();
    }

    public boolean canSkipClearance() {
        return Arm.getInstance().atCanSkipClearanceAngle() && Elevator.getInstance().atCanSkipClearanceHeight();
    }

    public static SuperStructureState getCorrespondingCoralScoreState(int level, boolean isFrontFacingReef) {
        switch (level) {
            case 1:
                return isFrontFacingReef ? SuperStructureState.L1_FRONT : SuperStructureState.L1_BACK;
            case 2:
                return isFrontFacingReef ? SuperStructureState.L2_FRONT : SuperStructureState.L2_BACK;
            case 3:
                return isFrontFacingReef ? SuperStructureState.L3_FRONT : SuperStructureState.L3_BACK;
            case 4:
                return isFrontFacingReef ? SuperStructureState.L4_FRONT : SuperStructureState.L4_BACK;
            default:
                return SuperStructureState.L1_FRONT;
        }
    }

    public static Command getCorrespondingCoralScoreStateCommand(int level, boolean isFrontFacingReef) {
        switch (level) {
            case 1:
                return isFrontFacingReef ? new SuperStructureCoralL1Front() : new SuperStructureCoralL1Back();
            case 2:
                return isFrontFacingReef ? new SuperStructureCoralL2Front() : new SuperStructureCoralL2Back();
            case 3:
                return isFrontFacingReef ? new SuperStructureCoralL3Front() : new SuperStructureCoralL3Back();
            case 4:
                return isFrontFacingReef ? new SuperStructureCoralL4Front() : new SuperStructureCoralL4Back();
            default:
                return new SuperStructureCoralL1Front();
        }
    }

    private void updateArmMotionProfileConstraints() {
        ArmState armState = getState().getArmState();
        if (armState == ArmState.FEED || armState == ArmState.PROCESSOR) {
            ShooterState shooterState = Shooter.getInstance().getState();
            if (shooterState == ShooterState.HOLD_ALGAE || shooterState == ShooterState.ACQUIRE_ALGAE) {
                arm.setMotionProfileConstraints(Settings.Arm.Constraints.MAX_VEL_BACK_TO_FEED_AND_PROCESSOR_WITH_ALGAE, Settings.Arm.Constraints.MAX_ACCEL_BACK_TO_FEED_AND_PROCESSOR_WITH_ALGAE);
            }
            else {
                arm.setMotionProfileConstraints(Settings.Arm.Constraints.DEFAULT_MAX_VEL_BACK_TO_FEED, Settings.Arm.Constraints.DEFAULT_MAX_ACCEL_BACK_TO_FEED);
            }
        }
        else if (armState == ArmState.CATAPULT_SHOOT) {
            arm.setMotionProfileConstraints(Settings.Arm.Constraints.MAX_VEL_CATAPULT, Settings.Arm.Constraints.MAX_ACCEL_CATAPULT);
        }
        else if (Robot.getMode() == RobotMode.AUTON) {
            arm.setMotionProfileConstraints(Settings.Arm.Constraints.MAX_VEL_AUTON, Settings.Arm.Constraints.MAX_ACCEL_AUTON);
        }
        else {
            arm.setMotionProfileConstraints(Settings.Arm.Constraints.MAX_VEL_TELEOP, Settings.Arm.Constraints.MAX_ACCEL_TELEOP);
        }
    }

    private void updateElevatorMotionProfileConstraints() {
        if (Robot.getMode() == RobotMode.AUTON) {
            elevator.setMotionProfileConstraints(Settings.Elevator.Constraints.MAX_VELOCITY_METERS_PER_SECOND_AUTON, Settings.Elevator.Constraints.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_AUTON);
        }
        else {
            elevator.setMotionProfileConstraints(Settings.Elevator.Constraints.MAX_VELOCITY_METERS_PER_SECOND_TELEOP, Settings.Elevator.Constraints.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_TELEOP);
        }
    }
}
