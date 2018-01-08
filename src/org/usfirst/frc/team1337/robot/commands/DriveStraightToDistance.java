package org.usfirst.frc.team1337.robot.commands;

import org.usfirst.frc.team1337.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *
 */
public class DriveStraightToDistance extends Command {
	private PIDController gyroPID;
	double counter = 0;
	double timeout = 0;
	double mySetpoint;
	// Probably needs to be tuned
	private double kPG = 0.055, kIG = 0.0, kDG = 0.055; // Gyro PID
	// private double kPE = 0.001, kIE = 0.00, kDE = 0; // Encoder PID

	private double rotate;
    @SuppressWarnings("deprecation")
	public DriveStraightToDistance(double ticks, double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	timeout = time;
		requires(Robot.driveSubsystem);
		mySetpoint = ticks;
		gyroPID = new PIDController(kPG, kIG, kDG, new PIDSource() {
			PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				m_sourceType = pidSource;
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return m_sourceType;
			}

			@Override
			public double pidGet() {
				//return Robot.driveSubsystem angle
				return 0.0;
			}
		}, new PIDOutput() {
			@Override
			public void pidWrite(double pidRotate) {
				rotate = pidRotate;
			}
		});
		gyroPID.setAbsoluteTolerance(0.05);
		gyroPID.setSetpoint(0);

		LiveWindow.addActuator("Drive Gyro", "Gyro PID Tuning", gyroPID);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
    }
}
