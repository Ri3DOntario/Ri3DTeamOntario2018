package org.usfirst.frc.team1337.robot.subsystems;

import java.util.Vector;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.RobotMap;
import org.usfirst.frc.team1337.robot.commands.DriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveSubsystem extends Subsystem {
	Encoder leftEnc, rightEnc;
	WPI_TalonSRX rightMaster, rightSlave, leftMaster, leftSlave;
	//Solenoid gearShiftSolenoid;
	SpeedControllerGroup leftDrive, rightDrive;
	DifferentialDrive myDrive;
	PIDController encoderPID;
	double speed;
	private double kPE = 0.0, kIE = 0.0, kDE = 0.0; // Encoder PID
	public double reverse = 1;

	public int reverseTimer = 0;
	public DriveSubsystem() {
		try {
		//initialize gyro
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating gyro " + ex.getMessage(), true);
		}

		rightMaster = new WPI_TalonSRX(RobotMap.frontRightDrive);
		leftMaster = new WPI_TalonSRX(RobotMap.frontLeftDrive);
		rightSlave = new WPI_TalonSRX(RobotMap.backRightDrive);
		leftSlave = new WPI_TalonSRX(RobotMap.backLeftDrive);
		
		leftEnc = new Encoder(0,1);
		rightEnc=new Encoder(2,3);
	/*	leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
		rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());*/
		
		leftDrive=new SpeedControllerGroup(leftMaster, leftSlave);
		rightDrive = new SpeedControllerGroup(rightMaster, rightSlave);
		
		myDrive = new DifferentialDrive(leftDrive, rightDrive);
		rightDrive.setInverted(false);
		leftDrive.setInverted(true);

		//rightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
		//leftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
		//rightMaster.setPosition(0);
		//leftMaster.setPosition(0);
		//gearShiftSolenoid = new Solenoid(0);

		//rightMaster.setProfile(0);
		//leftMaster.setProfile(0);

		// rightMaster.setPID(2.3, 0, 23);
		// leftMaster.setPID(2.1, 0, 21);

		//leftMaster.configPeakOutputVoltage(+12f, -12f);
		//rightMaster.configPeakOutputVoltage(+12f, -12f);

		//leftMaster.setNeutralMode(false);
		//rightMaster.setNeutralMode(false);
	}
	
	public void joystickDrive(double x, double y) {
	reverseTimer++;
	/*	if(Robot.oi.joystick1.getRawButton(10) && reverse == 1) {
			reverse = -1;
		} else if (Robot.oi.joystick1.getRawButton(10) && reverse == -1) {
			reverse = 1;
		}*/
		
		
	
	myDrive.curvatureDrive(y*reverse, x, true);
	//	myDrive.arcadeDrive(y, x, true);
		if (Robot.oi.joystick1.getRawButton(1)) {
			shiftGears(true);
		} else {
			shiftGears(false);
		}
	}
	
	public void resetEnc() {
	leftEnc.reset();
	rightEnc.reset();

	}
	
	public void zeroGyro() {
	}
	
	public void shiftGears(boolean shift) {
	//	gearShiftSolenoid.set(shift);
	}
	
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveCommand());
    }
    
    public void arcadeDrive(double speed, double rotate) {
		myDrive.arcadeDrive(speed, rotate);
	}


	public void Setpoint(double ticks) {
		encoderPID.setSetpoint(ticks);
	}

	public double sendSpeed() {
		speed = Math.max(Math.min(0.8, speed), -0.8); // the double value can be

		return speed;
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
				return 0.0; //encoder distance
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
    public void logging() {
    	SmartDashboard.putNumber("left encoder", leftEnc.getDistance());
    	SmartDashboard.putNumber("right encoder", rightEnc.getDistance());
    }
}

