package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends IterativeRobot {
	

	//sides for switch in autonomous 
   String sides;
	
	//Drive train and channels
	DifferentialDrive myRobot;
	
	//Drive channels (2)
	final int kLeftChannel = 1;
	final int kRightChannel = 0;
	
	//Constructs values for driving
	double motorXValue;
	double motorYValue;
	double iterationY;
	double iterationX;
	double absYValue;
	
	//Controller
	XboxController controller;
	
	//Talons (4)
	Talon elevatorMotor = new Talon(2);
	Talon armWheels = new Talon (3);
	//Talon climbingMotor = new Talon(4);
	
	//Solenoids (4)
	Solenoid boxClamp = new Solenoid(0);
	Solenoid boxOpen = new Solenoid(1);
	Solenoid frameRetraction = new Solenoid(2);
	Solenoid frameExtension = new Solenoid(3);
	
	//Limiter switches (4)
	DigitalInput upperLimit = new DigitalInput(0);
	DigitalInput lowerLimit = new DigitalInput(1);
	DigitalInput magUpperLimit = new DigitalInput(2);
	DigitalInput magLowerLimit = new DigitalInput(3);

	 
  Command autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();
	
	//Timer
	Timer timer = new Timer();
	
	//Booleans for the macros
	boolean frameRe, frameEx, boxCl, boxOp, elevatorUp, elevatorDwn;

	@Override
	public void robotInit() {
		
		//Left speed controllers
    Talon left = new Talon(kLeftChannel);
    left.setInverted(true);
		SpeedControllerGroup leftGroup = new SpeedControllerGroup(left);
		
		//Right speed controllers
		Talon right = new Talon(kRightChannel);
		SpeedControllerGroup rightGroup = new SpeedControllerGroup(right);
		
		//Construct drive
		myRobot = new DifferentialDrive(leftGroup, rightGroup);
		
		//Construct controller
		controller = new XboxController(0);
    
    CameraServer.getInstance().startAutomaticCapture();
		
			
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {	
		
		//Calculate motor values
		if (controller.getY(Hand.kLeft) != 0) {
			
			iterationY = iterationY + 0.2;
			absYValue = Math.abs(controller.getY(Hand.kLeft));
			motorYValue = (absYValue)/(1+((absYValue-(0.001))/(0.001))*(Math.pow(2.8, (-(0.5)*(iterationY)))));
			
			if(controller.getY(Hand.kLeft) < 0) motorYValue = -(motorYValue);
			
		} else {
			
			motorYValue = 0;
			iterationY = 0;
			absYValue = 0;
		
		}
		
		motorXValue = controller.getX(Hand.kLeft) * .99;
		
		
		
		//Ultrasonic stuff
		//if (motorXValue > 0 && sonic.getRangeMM() < 500) motorXValue = 0;
		
		//Timer delay to avoid hogging CPU cycles (maybe redundant)
		//Timer.delay(0.005);
		
		//Arcade drive based on calculated motor values
		myRobot.arcadeDrive(-(controller.getRawAxis(0)), (controller.getRawAxis(1)), true);
		
		
		//Move elevator motor based on limiter switches and position of Right Analog Stick
	//	if (upperLimit.get() && elevatorUp) elevatorMotor.set(1);
	//	else if (lowerLimit.get() && elevatorDwn) elevatorMotor.set(-1);
	//	else if (upperLimit.get() && controller.getY(Hand.kRight) > 0) elevatorMotor.set(controller.getY(Hand.kRight));
	//	else if (lowerLimit.get() && controller.getY(Hand.kRight) < 0) elevatorMotor.set(controller.getY(Hand.kRight));
	//	else elevatorMotor.set(0);
		
    // //Extends the arm on Y Button, retracts (climbs) on X Button
    double elevatorPower = controller.getRawAxis(3);

    // if (magUpperLimit.get() || magLowerLimit.get()) {
      // elevatorMotor.set(0);
     //}
     //else {
      elevatorMotor.set(-elevatorPower*0.5);
    // }
		//  if (controller.getYButton() && magUpperLimit.get()) {
		//  	elevatorMotor.set(1);
		//  }
		//  else if (controller.getXButton() && magLowerLimit.get()) {
		//  	elevatorMotor.set(-1);
		//  }
		//  else {	
		//  	elevatorMotor.set(0);
		//  }
		
		//Extends frame on B Button, Retracts frame on A Button
		if (controller.getRawButton(2) ) {
			frameRetraction.set(false);
			frameExtension.set(true);
		}
		else if (controller.getRawButton(3)) {
			frameExtension.set(false);
			frameRetraction.set(true);
		}
		else {
			frameRetraction.set(false);
			frameExtension.set(false);
		}
	
		//Clamps down on Right Trigger, opens up on Left Trigger
		if (controller.getRawButton(1)){
			boxOpen.set(false);
			boxClamp.set(true);
		}
		else if (controller.getRawButton(4)){
			boxClamp.set(false);
			boxOpen.set(true);
		}
		else {
			boxClamp.set(false);
			boxOpen.set(false);			
		}
		
		//Left bumper pulls in box, Right bumper shoots it out
		if(controller.getBumper(Hand.kRight))
			armWheels.set(1);
		else if(controller.getBumper(Hand.kLeft))
			armWheels.set(-1);
		else
			armWheels.set(0);
		
	}

	@Override
	public void teleopInit() {
		
    if (autonomousCommand != null){
      autonomousCommand.cancel();
    }
	}
	@Override
	public void teleopPeriodic() {	
		
		//Calculate motor values
		if (controller.getY(Hand.kLeft) != 0) {
			
			iterationY = iterationY + 0.2;
			absYValue = Math.abs(controller.getY(Hand.kLeft));
			motorYValue = (absYValue)/(1+((absYValue-(0.001))/(0.001))*(Math.pow(2.8, (-(0.5)*(iterationY)))));
			
			if(controller.getY(Hand.kLeft) < 0) motorYValue = -(motorYValue);
			
		} else {
			
			motorYValue = 0;
			iterationY = 0;
			absYValue = 0;
		
		}
		
		motorXValue = controller.getX(Hand.kLeft) * .99;
		
		
		
		//Ultrasonic stuff
		//if (motorXValue > 0 && sonic.getRangeMM() < 500) motorXValue = 0;
		
		//Timer delay to avoid hogging CPU cycles (maybe redundant)
		//Timer.delay(0.005);
		
		//Arcade drive based on calculated motor values
		myRobot.arcadeDrive(-(controller.getRawAxis(0)), (controller.getRawAxis(1)), true);
		
		
		//Move elevator motor based on limiter switches and position of Right Analog Stick
	//	if (upperLimit.get() && elevatorUp) elevatorMotor.set(1);
	//	else if (lowerLimit.get() && elevatorDwn) elevatorMotor.set(-1);
	//	else if (upperLimit.get() && controller.getY(Hand.kRight) > 0) elevatorMotor.set(controller.getY(Hand.kRight));
	//	else if (lowerLimit.get() && controller.getY(Hand.kRight) < 0) elevatorMotor.set(controller.getY(Hand.kRight));
	//	else elevatorMotor.set(0);
		
    // //Extends the arm on Y Button, retracts (climbs) on X Button
    double elevatorPower = controller.getRawAxis(3);

    // if (magUpperLimit.get() || magLowerLimit.get()) {
      // elevatorMotor.set(0);
     //}
     //else {
      elevatorMotor.set(-elevatorPower*0.5);
    // }
		//  if (controller.getYButton() && magUpperLimit.get()) {
		//  	elevatorMotor.set(1);
		//  }
		//  else if (controller.getXButton() && magLowerLimit.get()) {
		//  	elevatorMotor.set(-1);
		//  }
		//  else {	
		//  	elevatorMotor.set(0);
		//  }
		
		//Extends frame on B Button, Retracts frame on A Button
		if (controller.getRawButton(2) ) {
			frameRetraction.set(false);
			frameExtension.set(true);
		}
		else if (controller.getRawButton(3)) {
			frameExtension.set(false);
			frameRetraction.set(true);
		}
		else {
			frameRetraction.set(false);
			frameExtension.set(false);
		}
	
		//Clamps down on Right Trigger, opens up on Left Trigger
		if (controller.getRawButton(1)){
			boxOpen.set(false);
			boxClamp.set(true);
		}
		else if (controller.getRawButton(4)){
			boxClamp.set(false);
			boxOpen.set(true);
		}
		else {
			boxClamp.set(false);
			boxOpen.set(false);			
		}
		
		//Left bumper pulls in box, Right bumper shoots it out
		if(controller.getBumper(Hand.kRight))
			armWheels.set(1);
		else if(controller.getBumper(Hand.kLeft))
			armWheels.set(-1);
		else
			armWheels.set(0);
		
	}

	@Override
	public void testInit() {
		
		System.out.println("Entering Test Mode:");
	}
	
	
	
	@Override
	public void testPeriodic() { 
		
	}}
		