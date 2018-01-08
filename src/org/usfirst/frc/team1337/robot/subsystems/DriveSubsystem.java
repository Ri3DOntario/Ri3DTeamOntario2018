package org.usfirst.frc.team1337.robot.subsystems;

import java.util.Vector;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.RobotMap;
import org.usfirst.frc.team1337.robot.commands.DriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSubsystem extends Subsystem {
	ADXRS450_Gyro gyro;
	Ultrasonic sonic;
	Encoder leftEnc, rightEnc;
	WPI_TalonSRX rightMaster, rightSlave, leftMaster, leftSlave;
	// Solenoid gearShiftSolenoid;
	SpeedControllerGroup leftDrive, rightDrive;
	DifferentialDrive myDrive;
	PIDController encoderPID;
	double speed;
	//needs to be tuned
	private double kPE = 0.0, kIE = 0.0, kDE = 0.0; // Encoder PID
	public double reverse = 1;

	public DriveSubsystem() {
		try {
			gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating gyro " + ex.getMessage(), true);
		}
		sonic = new Ultrasonic(0, 1);

		rightMaster = new WPI_TalonSRX(RobotMap.frontRightDrive);
		leftMaster = new WPI_TalonSRX(RobotMap.frontLeftDrive);
		rightSlave = new WPI_TalonSRX(RobotMap.backRightDrive);
		leftSlave = new WPI_TalonSRX(RobotMap.backLeftDrive);

		leftEnc = new Encoder(0, 1, false, EncodingType.k4X);
		rightEnc = new Encoder(2, 3, false, EncodingType.k4X);

		leftDrive = new SpeedControllerGroup(leftMaster, leftSlave);
		rightDrive = new SpeedControllerGroup(rightMaster, rightSlave);

		myDrive = new DifferentialDrive(leftDrive, rightDrive);
		rightDrive.setInverted(false);
		leftDrive.setInverted(true);

		// gearShiftSolenoid = new Solenoid(0);

		// leftMaster.configPeakOutputVoltage(+12f, -12f);
		// rightMaster.configPeakOutputVoltage(+12f, -12f);

		// leftMaster.setNeutralMode(false);
		// rightMaster.setNeutralMode(false);
	}

	public void joystickDrive(double x, double y) {
		/*
		 * if(Robot.oi.joystick1.getRawButton(10) && reverse == 1) { reverse = -1; }
		 * else if (Robot.oi.joystick1.getRawButton(10) && reverse == -1) { reverse = 1;
		 * }
		 * 
		 */

		myDrive.curvatureDrive(y * reverse, x, true);
		// myDrive.arcadeDrive(y, x, true);
		if (Robot.oi.joystick1.getRawButton(1))
			shiftGears(true);
		else
			shiftGears(false);
		logging();
	}

	public void resetEnc() {
		leftEnc.reset();
		rightEnc.reset();
	}

	public void zeroGyro() {
		gyro.reset();
	}

	public void shiftGears(boolean shift) {
		// gearShiftSolenoid.set(shift);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveCommand());
	}

	public void arcadeDrive(double speed, double rotate) {
		myDrive.arcadeDrive(speed, rotate);
	}

	public void Setpoint(double ticks) {
		encoderPID.setSetpoint(ticks);
	}

	public double sendSpeed() {
		return Math.max(Math.min(0.8, speed), -0.8); // the double value can be
	}

	public void resetEncPID() {
		encoderPID.reset();
	}

	public boolean encOnTarget() {
		return encoderPID.onTarget();
	}

	public void disableEncPID() {
		encoderPID.disable();
	}

	public void initEncPID() {
		encoderPID = new PIDController(kPE, kIE, kDE, new PIDSource() {
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

	public void enableEncPID() {
		encoderPID.enable();
	}

	public double getGyro() {
		return gyro.getAngle();
	}

	public void logging() {
		SmartDashboard.putNumber("left encoder", leftEnc.getDistance());
		SmartDashboard.putNumber("right encoder", rightEnc.getDistance());
		SmartDashboard.putNumber("gyro", gyro.getAngle());
		SmartDashboard.putNumber("ultrasonic", sonic.getRangeMM());
		
	}
}
