package org.usfirst.frc.team1337.robot.subsystems;

import java.util.Vector;
import org.usfirst.frc.team1337.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveSubsystem extends Subsystem {

    RobotDrive robotDrive;
    
    TalonSRX backLeftDrive = new TalonSRX(RobotMap.backLeftDrive);
    TalonSRX frontLeftDrive = new TalonSRX(RobotMap.frontLeftDrive);
    TalonSRX backRightDrive = new TalonSRX(RobotMap.backRightDrive);
    TalonSRX frontRightDrive = new TalonSRX(RobotMap.frontRightDrive);
	
//    public ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    DoubleSolenoid gearShift = new DoubleSolenoid(RobotMap.PCM, RobotMap.shotPinForward, RobotMap.shotPinReverse);
    
	public DriveSubsystem() {
		robotDrive = new RobotDrive(RobotMap.frontLeftDrive,RobotMap.backLeftDrive,RobotMap.frontRightDrive,RobotMap.backRightDrive);
		
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
		
		robotDrive.setSafetyEnabled(true);
	}
	
	public void teleMove(Joystick joystick, double speed) {
		robotDrive.arcadeDrive(joystick.getY()*speed, joystick.getX()*speed);
	}
	
	public void autoMove(double driveSpeed, double rotSpeed) {
		robotDrive.arcadeDrive(driveSpeed, rotSpeed);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

