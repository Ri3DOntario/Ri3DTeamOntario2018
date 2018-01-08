package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.commands.Intake;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intaker extends Subsystem {

	Victor leftIntake, rightIntake;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public Intaker() {
		leftIntake = new Victor(0);
		rightIntake = new Victor(1);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new Intake());
	}

	public void set(double speed) {
		leftIntake.set(speed);
		rightIntake.set(-speed);
	}
	
	public void logging() {
		SmartDashboard.putNumber("Intake speed", leftIntake.get());
	}
}
