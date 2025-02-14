package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

import edu.wpi.first.math.geometry.Rotation2d;
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

    protected SuperStructure() {
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

    private boolean canMoveArmToNextTarget() {
        ArmState targetArmState = getTargetState().getTargetArmState();

        Rotation2d currentArmAngle = arm.getCurrentAngle();
        double currentElevatorHeight = elevator.getCurrentHeight();

        return (targetArmState.getTargetAngle().getDegrees() >= 0 && currentArmAngle.getDegrees() >= 0)
            || (currentElevatorHeight >= Constants.Elevator.FUNNEL_CLEAR_HEIGHT);
    }

    private boolean canMoveElevatorToNextTarget() {
        ElevatorState targetElevatorState = getTargetState().getTargetElevatorState();

        Rotation2d currentArmAngle = arm.getCurrentAngle();
        
        return (currentArmAngle.getDegrees() >= 0)
            || (targetElevatorState.getTargetHeight() >= Constants.Elevator.FUNNEL_CLEAR_HEIGHT && currentArmAngle.getDegrees() >= 0);
    }

    private boolean mustRaiseElevatorFirst() {
        ArmState targetArmState = getTargetState().getTargetArmState();

        Rotation2d currentArmAngle = arm.getCurrentAngle();
        double currentElevatorHeight = elevator.getCurrentHeight();

        return (currentArmAngle.getDegrees() > Constants.Arm.MIN_ANGLE_TO_CLEAR_FUNNEL.getDegrees() 
                && targetArmState.getTargetAngle().getDegrees() < Constants.Arm.MIN_ANGLE_TO_CLEAR_FUNNEL.getDegrees()
                && currentElevatorHeight < Constants.Elevator.FUNNEL_CLEAR_HEIGHT
                )
            || (currentElevatorHeight < Constants.Elevator.FUNNEL_CLEAR_HEIGHT
                && targetArmState.getTargetAngle().getDegrees() < Constants.Arm.MIN_ANGLE_TO_CLEAR_FUNNEL.getDegrees()
                );
    }

    @Override
    public void periodic() {
        ArmState currentArmState = arm.getState();
        ArmState targetArmState = getTargetState().getTargetArmState();

        ElevatorState currentElevatorState = elevator.getState();
        ElevatorState targetElevatorState = getTargetState().getTargetElevatorState();

        if (targetArmState != currentArmState || targetElevatorState != currentElevatorState) {
            if (canMoveArmToNextTarget() && elevator.atTargetHeight()) {
                arm.setState(targetArmState);
            }
            else if (canMoveElevatorToNextTarget() && arm.atTargetAngle()) {
                elevator.setState(targetElevatorState);
            }
            else if (mustRaiseElevatorFirst()) {
                elevator.setState(ElevatorState.CLEAR_FUNNEL);
            }
        }

        visualizer.update(elevator.getCurrentHeight(), arm.getCurrentAngle());
    }
}
