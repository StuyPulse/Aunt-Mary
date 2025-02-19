package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{
    private static final SuperStructure instance;
    
    static { 
        instance = new SuperStructure();
    }

    public static SuperStructure getInstance() {
        return instance;
    }

    public enum SuperStructureTargetState {
        FEED(ArmState.FEED, ElevatorState.FEED),
        STOW(ArmState.STOW, ElevatorState.STOW),
        L2_FRONT(ArmState.L2_FRONT, ElevatorState.L2_FRONT),
        L2_BACK(ArmState.L2_BACK, ElevatorState.L2_BACK),
        L3_FRONT(ArmState.L3_FRONT, ElevatorState.L3_FRONT),
        L3_BACK(ArmState.L3_BACK, ElevatorState.L3_BACK),
        L4_FRONT(ArmState.L4_FRONT, ElevatorState.L4_FRONT),
        L4_BACK(ArmState.L4_BACK, ElevatorState.L4_BACK),
        ALGAE_L2(ArmState.ALGAE_L2, ElevatorState.ALGAE_L2),
        ALGAE_L3(ArmState.ALGAE_L3, ElevatorState.ALGAE_L3),
        BARGE(ArmState.BARGE, ElevatorState.BARGE);

        private ArmState targetArmState;
        private ElevatorState targetElevatorState;

        private SuperStructureTargetState(ArmState targetArmState, ElevatorState targetElevatorState) {
            this.targetArmState = targetArmState;
            this.targetElevatorState = targetElevatorState;
        }

        public ArmState getTargetArmState() {
            return this.targetArmState;
        }

        public ElevatorState getTargetElevatorState() {
            return this.targetElevatorState;
        }
    }

    private SuperStructureTargetState targetState;

    private Arm arm;
    private Elevator elevator;

    private SuperStructureVisualizer visualizer;

    private SuperStructure() {
        this.targetState = SuperStructureTargetState.STOW;

        this.arm = Arm.getInstance();
        this.elevator = Elevator.getInstance();

        visualizer = SuperStructureVisualizer.getInstance();
    }

    public void setTargetState(SuperStructureTargetState state) {
        this.targetState = state;
    }

    public SuperStructureTargetState getTargetState() {
        return this.targetState;
    }

    public boolean isInScoreState() {
        switch (getTargetState()) {
            case L2_FRONT:
            case L2_BACK:
            case L3_FRONT:
            case L3_BACK:
            case L4_FRONT:
            case L4_BACK:
            case ALGAE_L2:
            case ALGAE_L3:
            case BARGE:
                return true;
            default:
                return false;
        }
    }

    private boolean canMoveDownFromBarge() {
        double distanceFromCenterLine = Math.abs(Field.LENGTH / 2 - CommandSwerveDrivetrain.getInstance().getPose().getX());
        return distanceFromCenterLine > Settings.CLEARANCE_DISTANCE_FROM_CENTERLINE_FOR_BARGE;
    }

    private void updateArmElevatorTargetStates() {
        ArmState currentArmState = arm.getState();
        ArmState targetArmState = getTargetState().getTargetArmState();

        ElevatorState currentElevatorState = elevator.getState();
        ElevatorState targetElevatorState = getTargetState().getTargetElevatorState();

        if (currentArmState == ArmState.BARGE && currentElevatorState == ElevatorState.BARGE && !canMoveDownFromBarge()) {
            return;
        }

        if (getTargetState() == SuperStructureTargetState.FEED) {
            if ((currentElevatorState == ElevatorState.FEED && elevator.atTargetHeight()) || !Settings.EnabledSubsystems.ELEVATOR.get()) {
                arm.setState(ArmState.FEED);
            }
            else if (((currentArmState == ArmState.VERTICAL_DOWN && arm.atTargetAngle()) || !Settings.EnabledSubsystems.ARM.get()) && currentElevatorState != ElevatorState.FEED) {
                elevator.setState(ElevatorState.FEED);
            }
            else {
                arm.setState(ArmState.VERTICAL_DOWN);
            }
        }
        else {
            if ((targetElevatorState != currentElevatorState && arm.atTargetAngle()) || !Settings.EnabledSubsystems.ARM.get()) {
                if (currentArmState == ArmState.FEED) {
                    arm.setState(ArmState.VERTICAL_DOWN);
                }
                else {
                    elevator.setState(targetElevatorState);
                }
            }
            else if ((targetArmState != currentArmState && elevator.atTargetHeight()) || !Settings.EnabledSubsystems.ELEVATOR.get()) {
                arm.setState(targetArmState);
            }
        }
    }

    @Override
    public void periodic() {
        updateArmElevatorTargetStates();

        visualizer.update(elevator.getCurrentHeight(), arm.getCurrentAngle());

        SmartDashboard.putString("Super Structure/Target Arm State", getTargetState().getTargetArmState().toString());
        SmartDashboard.putString("Super Structure/Target Elevator State", getTargetState().getTargetElevatorState().toString());
    }
}
