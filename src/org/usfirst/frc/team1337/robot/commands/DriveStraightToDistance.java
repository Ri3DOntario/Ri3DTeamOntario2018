package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightToDistance extends Command {
	double counter = 0, timeout, mySetpoint;
	// Probably needs to be tuned
	private double kPG = 0.055, kIG = 0.0, kDG = 0.055; // Gyro PID
	private double kPE = 0.001, kIE = 0.00, kDE = 0; // Encoder PID
	
	public DriveStraightToDistance(double ticks, double time) {
		requires(Robot.driveSubsystem);
		timeout = time;
		mySetpoint = ticks;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.driveSubsystem.initEncPID(kPE, kIE, kDE);
    		Robot.driveSubsystem.initGyroPID(kPG, kIG, kDG);
		Robot.driveSubsystem.encoderPID.reset();
		Robot.driveSubsystem.gyroPID.reset();
		Robot.driveSubsystem.encoderPID.setSetpoint(mySetpoint);
		Robot.driveSubsystem.gyroPID.setSetpoint(0);
		Robot.driveSubsystem.resetEnc();
		Robot.driveSubsystem.resetGyro();
		Robot.driveSubsystem.encoderPID.enable(); 
		Robot.driveSubsystem.gyroPID.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		Robot.driveSubsystem.arcadeDrive(Robot.driveSubsystem.scaleSpeedPID(), Robot.driveSubsystem.scaleRotatePID());

		counter++;
		logging();
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
		Robot.driveSubsystem.encoderPID.disable();
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
    		Robot.logCurrentCommand("DriveStraightToDistance");
    		Robot.driveSubsystem.logging();
    		SmartDashboard.putBoolean("On Target Enc", Robot.driveSubsystem.encoderPID.onTarget());
    		SmartDashboard.putBoolean("On Target Gyro", Robot.driveSubsystem.gyroPID.onTarget());
    }
}
