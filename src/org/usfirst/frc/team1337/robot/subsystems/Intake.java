package org.usfirst.frc.team1337.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {
Victor leftIntake, rightIntake;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
public Intake(){
	leftIntake = new Victor(0);
	rightIntake = new Victor(1);
}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void IntakeSet(double speed) {
    	leftIntake.set(speed);
    	rightIntake.set(-speed);
    }
}

