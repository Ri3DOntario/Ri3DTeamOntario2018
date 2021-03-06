package org.usfirst.frc.team1337.robot.subsystems;

import org.usfirst.frc.team1337.robot.Robot;
import org.usfirst.frc.team1337.robot.commands.PIDArm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends Subsystem {
	
//declare the motoro controller for the arm TalonSRX
public WPI_TalonSRX arm;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
public Arm() {
	arm = new WPI_TalonSRX(9);
	
	//set the feedback sensor to a absolute magnetic encoder
	arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 5);
	
	//these need to be tuned
	//kill me
	arm.config_kP(0, 0.0, 0);
	arm.config_kI(0, 0.0, 0);
	arm.config_kD(0, 0.0, 0);
	
	//arm.configForwardSoftLimitThreshold(0, 0);
	//arm.configReverseSoftLimitThreshold(0, 0);
	
	//confgiure software limits
	arm.configForwardSoftLimitEnable(false, 0);
	arm.configReverseSoftLimitEnable(false, 0);
	arm.setSensorPhase(true);
}

//the command will continuously run from the beginning unless interrupted 
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new PIDArm());
    }
 
    //oof this 
    public void ArmPID(double speed) {
     if(Robot.oi.joystick2.getRawButton(3)){ //intake
			arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick2.getRawButton(4)){ //scoring high scale
		arm.set(ControlMode.Position, 0);
	/*} else if (Robot.oi.joystick1.getRawButton(2)) {//climb position
		arm.set(ControlMode.Position, 0);
	} else if (Robot.oi.joystick1.getRawButton(1)) {//switch
		arm.set(ControlMode.Position, 0);*/
	} else {
		//why is it being limited to 70%??
		arm.set(ControlMode.PercentOutput, speed*0.70);
	}
 	logging();

    }
    
    //oof why is this called logging when it is not logging anything?
    //this will display live information on the smartdashboard
    public void logging() {
    	SmartDashboard.putNumber("arm position", arm.getSelectedSensorPosition(0)); //wtf is this
    	SmartDashboard.putNumber("arm set pos", arm.getActiveTrajectoryPosition()); //wtf is this also
    }
    //method to control arm during auto 
    public void AutoArm(double pos) {
    	arm.set(ControlMode.Position, pos);
    }
}

