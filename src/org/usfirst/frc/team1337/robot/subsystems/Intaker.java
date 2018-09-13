package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.commands.Intake;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intaker extends Subsystem {

	Victor leftIntake, rightIntake;
	Solenoid leftSol, rightSol;


	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public Intaker() {
		//Initialize the intake motor controllers
		leftIntake = new Victor(0);
		rightIntake = new Victor(1);
		
		//initialize the solenoids
		//IDs need to be updated
		leftSol = new Solenoid(4);
		rightSol = new Solenoid(5);
		
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new Intake());
	}
//control the intake wheel speed
	public void set(double speed) {
		leftIntake.set(speed);
		rightIntake.set(-speed);
	}
	
	public void logging() {
		SmartDashboard.putNumber("Intake speed", leftIntake.get());
	}
	//control the pneumatic cylinders to actuate the intake
	public void actuate(boolean act) {
		leftSol.set(act);
		rightSol.set(act);
	}
}
