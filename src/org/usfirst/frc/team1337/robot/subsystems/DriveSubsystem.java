package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.RobotMap;
import org.usfirst.frc.team1337.robot.commands.JoystickDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

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
	Compressor comp = new Compressor();
	final double MAX_SPEED = 0.8;
	ADXRS450_Gyro gyro;
	Ultrasonic sonic;
	Encoder leftEnc, rightEnc;
	WPI_TalonSRX rightMaster, rightSlave, leftMaster, leftSlave;
	DoubleSolenoid gearShiftSolenoid;
	SpeedControllerGroup leftDrive, rightDrive;
	DifferentialDrive myDrive;
	PIDController encoderPID;
	double speed;
	// needs to be tuned
	private double kPE = 0.01, kIE = 0.0001, kDE = 0.1; // Encoder PID
	public double reverse = 1;

	public DriveSubsystem() {
		
		//try starting up the gyro 
		try {
			gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
		} catch (RuntimeException ex) { //don't kms 
			DriverStation.reportError("Error instantiating gyro " + ex.getMessage(), true);
		}
		
		//do we even need this?
		//idk why we even have it on the robot
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

		//initialize the solenoid for gear shifting
		gearShiftSolenoid = new DoubleSolenoid(4,5);

		leftMaster.configPeakOutputForward(1, 0); // full 12v, no timeout
		rightMaster.configPeakOutputReverse(1, 0); // full 12v, no timeout
		
	
		//Scale?
		leftMaster.configVoltageCompSaturation(12*1.0, 0); 
		rightMaster.configVoltageCompSaturation(12*1.0, 0);
		
	}

	public void joystickDrive(double x, double y) {
				
	
		if(Robot.oi.joystick2.getRawButton(6)) { //trigger off board compressor on/off
			comp.start();
		}else {
			comp.stop();
		}
		
		//chezy drive basically
		//can this be configured to be more optimized?
		myDrive.curvatureDrive(y * reverse, x, true);
		
		if (Robot.oi.joystick1.getRawButton(6))
			shiftGears(true);
		else
			shiftGears(false);
		logging();
	}

	public void resetEnc() {
		leftEnc.reset();
		rightEnc.reset();
	}

	public void resetGyro() {
		gyro.reset();
	}

	//shift gears
	public void shiftGears(boolean shift) {
		if(shift) {
			gearShiftSolenoid.set(DoubleSolenoid.Value.kForward);
		}else {
			gearShiftSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
	}

	//this command is always running
	public void initDefaultCommand() {
		setDefaultCommand(new JoystickDrive());
	}

	//lets switch to chezy drive at some point
	//oh, do we even use this? Can it be commented out?
	//I think this is for auto
	public void arcadeDrive(double speed, double rotate) {
		myDrive.arcadeDrive(speed, rotate);
	}
	
	public void stopMotors() {
		arcadeDrive(0,0);
	}
	
	public void Setpoint(double ticks) {
		encoderPID.setSetpoint(ticks);
	}


	public double scaleSpeedPID() {
		return Math.max(Math.min(MAX_SPEED, speed), -MAX_SPEED);
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

	//PIDs
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
			//I want to die Jed pls explain
		});

		//how many inches is this?
		encoderPID.setAbsoluteTolerance(50); // might need to be tuned if command never ends
	
	}

	public void enableEncPID() {
		//enable the PID 
		encoderPID.enable();
	}

	//get angle in degrees
	public double getGyro() {
		return gyro.getAngle();
	}

	//not really logging, more just displaying information live on the smartdashboard
	public void logging() {
		SmartDashboard.putNumber("left encoder", leftEnc.getDistance());
		SmartDashboard.putNumber("right encoder", rightEnc.getDistance());
		SmartDashboard.putNumber("gyro", gyro.getAngle());
		SmartDashboard.putNumber("ultrasonic", sonic.getRangeMM());
	}
}
