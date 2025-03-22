package com.stuypulse.robot.subsystems.superStructure;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;

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
        L1(ElevatorState.L1, ArmState.L1),
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
            || state == SuperStructureState.L1;
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
                return SuperStructureState.L1;
            case 2:
                return isFrontFacingReef ? SuperStructureState.L2_FRONT : SuperStructureState.L2_BACK;
            case 3:
                return isFrontFacingReef ? SuperStructureState.L3_FRONT : SuperStructureState.L3_BACK;
            case 4:
                return isFrontFacingReef ? SuperStructureState.L4_FRONT : SuperStructureState.L4_BACK;
            default:
                return SuperStructureState.L1;
        }
    }

    private void updateArmMotionProfileConstraints() {
        ArmState armState = getState().getArmState();
        if (armState == ArmState.FEED) {
            arm.setMotionProfileConstraints(Settings.Arm.Constraints.MAX_VEL_BACK_TO_FEED, Settings.Arm.Constraints.MAX_ACCEL_BACK_TO_FEED);
        }
        else if (armState == ArmState.CLIMB) {
            arm.setMotionProfileConstraints(Settings.Arm.Constraints.MAX_VEL_CLIMB, Settings.Arm.Constraints.MAX_ACCEL_CLIMB);
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
