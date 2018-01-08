package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.OI;
import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickDrive extends Command {

    public JoystickDrive() {
    		requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		logging();
    		Robot.driveSubsystem.joystickDrive(Robot.oi.joystick1.getRawAxis(OI.J1_LEFT_STICK_Y_AXIS), Robot.oi.joystick1.getRawAxis(OI.J1_RIGHT_STICK_X_AXIS));   	
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
    		Robot.logCurrentCommandAndSubsystem("JoystickDrive");
    		Robot.driveSubsystem.logging();
    }
}
