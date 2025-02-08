package com.stuypulse.robot.commands.auton.routines;
import com.stuypulse.robot.commands.lokishooter.ShooterAcquireCoral;
import com.stuypulse.robot.commands.lokishooter.ShooterStop;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class HandoffRoutine extends SequentialCommandGroup {

    public HandoffRoutine() {
        
        addCommands(

            new ShooterAcquireCoral(), // make this based on sensor value later
            new ShooterStop()

        );

    }

}