package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.RobotMap;
import org.usfirst.frc.team1337.robot.commands.JoystickDrive;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSubsystem extends Subsystem {
	final double MAX_SPEED = 0.8;
	ADXRS450_Gyro gyro;
	Ultrasonic sonic;
	Encoder leftEnc, rightEnc;
	WPI_TalonSRX rightMaster, rightSlave, leftMaster, leftSlave;
	// Solenoid gearShiftSolenoid;
	SpeedControllerGroup leftDrive, rightDrive;
	DifferentialDrive myDrive;
	public double reverse = 1;
	
	public PIDController encoderPID, gyroPID;
	double speed, rotate;

	public DriveSubsystem() {
		try {
			gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating gyro " + ex.getMessage(), true);
		}
		sonic = new Ultrasonic(RobotMap.ULTRASONIC_PING, RobotMap.ULTRASONIC_ECHO);

		rightMaster = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MASTER);
		leftMaster = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MASTER);
		rightSlave = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE);
		leftSlave = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_SLAVE);

		leftEnc = new Encoder(RobotMap.LEFT_DRIVE_ENC_A, RobotMap.LEFT_DRIVE_ENC_B, false, EncodingType.k4X);
		rightEnc = new Encoder(RobotMap.RIGHT_DRIVE_ENC_A, RobotMap.RIGHT_DRIVE_ENC_B, false, EncodingType.k4X);

		leftDrive = new SpeedControllerGroup(leftMaster, leftSlave);
		rightDrive = new SpeedControllerGroup(rightMaster, rightSlave);

		myDrive = new DifferentialDrive(leftDrive, rightDrive);
		rightDrive.setInverted(false);
		leftDrive.setInverted(true);

		// gearShiftSolenoid = new Solenoid(0);

		leftMaster.configPeakOutputForward(1, 0); // full 12v, no timeout
		rightMaster.configPeakOutputReverse(1, 0); // full 12v, no timeout
	}

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickDrive());
	}
	
	public void joystickDrive(double x, double y) {
		/*if (Robot.oi.joystick1.getRawButtonPressed(10))
			reverse *= -1;*/
		myDrive.curvatureDrive(y * reverse, x, true);
		if (Robot.oi.joystick1.getRawButton(1))
			shiftGears(true);
		else
			shiftGears(false);
		logging();
	}

	public void arcadeDrive(double speed, double rotate) {
		myDrive.arcadeDrive(speed, rotate);
	}
	
	public void shiftGears(boolean shift) {
		// gearShiftSolenoid.set(shift);
	}
	
	public void resetEnc() {
		leftEnc.reset();
		rightEnc.reset();
	}

	public void resetGyro() {
		gyro.reset();
	}
	
	public void stopMotors() {
		arcadeDrive(0,0);
	}
	
	public void initEncPID(double p, double i, double d) {
		encoderPID = new PIDController(p, i, d, new PIDSource() {
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
				return leftEnc.getDistance();
			}
		}, new PIDOutput() {
			@Override
			public void pidWrite(double pidSpeed) {
				speed = pidSpeed;
			}
		});

		encoderPID.setAbsoluteTolerance(25); // might need to be tuned if command never ends
	}
	
	public double scaleSpeedPID() {
		return Math.max(Math.min(MAX_SPEED, speed), -MAX_SPEED);
	}
	
	public double getGyro() {
		return gyro.getAngle();
	}
	
	public void initGyroPID(double p, double i, double d) {
		gyroPID = new PIDController(p, i, d, new PIDSource() {
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
		
		gyroPID.setAbsoluteTolerance(0.5); // might need to be tuned if it never ends
		gyroPID.setInputRange(-180.0f, 180.0f);
		gyroPID.setOutputRange(-1.0, 1.0);
		gyroPID.setContinuous(true);
	}
	
	public double scaleRotatePID() {
		return Math.max(Math.min(MAX_SPEED, rotate), -MAX_SPEED);
	}
	
	public void logging() {
		SmartDashboard.putNumber("left encoder", leftEnc.getDistance());
		SmartDashboard.putNumber("right encoder", rightEnc.getDistance());
		SmartDashboard.putNumber("gyro", gyro.getAngle());
		SmartDashboard.putNumber("ultrasonic", sonic.getRangeMM());
	}
}
