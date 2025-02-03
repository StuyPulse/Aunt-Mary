package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand; 
import edu.wpi.first.wpilibj2.command.Command;  

public class ElevatorToHeight extends InstantCommand {

    public static Command untilDone(double height) {
        return new ElevatorToHeight(height)
            .andThen(new WaitUntilCommand(() -> Elevator.getInstance().atTargetHeight()));
    }
    
    private final Elevator elevator;
    private final double targetHeight;

    public ElevatorToHeight(double targetHeight){
        elevator = Elevator.getInstance();
        this.targetHeight = targetHeight;

        addRequirements(elevator);
    }

    public void initialize(){
        elevator.setTargetHeight(targetHeight);
    }
}
