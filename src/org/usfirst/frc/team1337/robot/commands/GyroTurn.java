package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class GyroTurn extends Command {
	final double MAX_SPEED = 0.8;
	double counter = 0, timeout, mySetpoint;
	// Probably needs to be tuned
	private double kPG = 0.05, kIG = 0.0005, kDG = 0.05; // Gyro PID

	public GyroTurn(double angle, double time) {
		requires(Robot.driveSubsystem);
		timeout = time;
		mySetpoint = angle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveSubsystem.initGyroPID(kPG, kIG, kDG);
		Robot.driveSubsystem.gyroPID.reset();
		Robot.driveSubsystem.gyroPID.setSetpoint(mySetpoint);
		Robot.driveSubsystem.resetGyro();
		Robot.driveSubsystem.gyroPID.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		logging();
		counter++;
		Robot.driveSubsystem.arcadeDrive(0, Robot.driveSubsystem.scaleRotatePID());		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (counter >= timeout)
			return true;
		else
			return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		logging();
		Robot.driveSubsystem.gyroPID.disable();
		Robot.driveSubsystem.stopMotors();
		Robot.driveSubsystem.resetEnc();
		Robot.driveSubsystem.resetGyro();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
	
	void logging() {
		Robot.logCurrentCommand("GyroTurn");
		Robot.driveSubsystem.logging();
		SmartDashboard.putBoolean("gyroTurnPID.onTarget()", Robot.driveSubsystem.gyroPID.onTarget());
		SmartDashboard.putNumber("Gyro Error", Robot.driveSubsystem.gyroPID.getError());
	}
}