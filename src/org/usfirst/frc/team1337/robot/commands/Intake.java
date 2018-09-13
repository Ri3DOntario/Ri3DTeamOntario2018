package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Intake extends Command {
double leftIn, rightIn;

	public Intake() {
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		leftIn = Robot.oi.joystick2.getRawAxis(2);
		rightIn = Robot.oi.joystick2.getRawAxis(3);

		logging();
		Robot.intake.set(leftIn-rightIn);
		if (Math.abs(leftIn) > 0.10 || Math.abs(rightIn) > 0.10){
			Robot.intake.actuate(true);

		} else {
			Robot.intake.actuate(false);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	void logging() {
		Robot.logCurrentCommand("Intake");
		Robot.intake.logging();
	}
}
