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
	private PIDController gyroPID;
	double counter = 0;
	double timeout = 0;
	// Probably needs to be tuned
	private double kPG = 0.05, kIG = 0.0005, kDG = 0.05; // Gyro PID

	private double rotate;

	public GyroTurn(double angle, double time) {
		requires(Robot.driveSubsystem);
		timeout = time;
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
				return Robot.driveSubsystem.getGyro();
			}
		}, new PIDOutput() {
			@Override
			public void pidWrite(double pidRotate) {
				rotate = pidRotate;
			}
		});
		
		gyroPID.setAbsoluteTolerance(1); // might need to be tuned if it never ends
		gyroPID.setInputRange(-180.0f, 180.0f);
		gyroPID.setOutputRange(-1.0, 1.0);
		gyroPID.setContinuous(true);
		gyroPID.setSetpoint(angle);

		/* Add the PID Controller to the Test-mode dashboard, allowing manual */
		/* tuning of the Turn Controller's P, I and D coefficients. */
		/* Typically, only the P value needs to be modified. */
		LiveWindow.addActuator("Drive", "Gyro Only Tuning", gyroPID);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveSubsystem.resetEnc();
		Robot.driveSubsystem.resetGyro();

		gyroPID.reset();
		gyroPID.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		logging();
		counter++;
		rotate = Math.max(Math.min(MAX_SPEED, rotate), -MAX_SPEED); //scaling
		Robot.driveSubsystem.arcadeDrive(0, rotate);		
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
		gyroPID.disable();
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
		SmartDashboard.putBoolean("gyroTurnPID.onTarget()", gyroPID.onTarget());
		SmartDashboard.putNumber("Gyro Error", gyroPID.getError());
	}
}