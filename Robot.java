/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Pneumatics
  Compressor compress;
  boolean compressOn = true;
  
  //Solenoid(s)
  Solenoid hatchEjectOut;
  final int hEO = 3;
  Solenoid hatchEjectIn;
  final int hEI = 2;
  Solenoid hatchRotateOut;
  final int hRO = 1;
  Solenoid hatchRotateIn;
  final int hRI = 0;

  //Motors
  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide;
  DifferentialDrive drive;

  final int frontLeftChannel = 1;
  Spark frontLeft;
  final int backLeftChannel = 0;
  Spark backLeft;
  final int frontRightChannel = 3;
  Spark frontRight;
  final int backRightChannel = 2;
  Spark backRight;

  final int rampChannel = 4;
  Spark ramp;
  SpeedControllerGroup rampC;
  int rampT = 0;
  boolean rampOut = false;
  boolean rampLock = true;

  //Sensors
  ADXRS450_Gyro gyro;

  final int driveN = 0;
  XboxController driveController = new XboxController(driveN);

  //Acceleration and Such Tracker
  double lastSpeedL = 0;
  double lastSpeedR = 0;

  public double accelCont(double speedNow, double speedLast){
    double c = speedNow - speedLast;
    double maxC = 0.075;
    if(c > maxC){
      return maxC + speedLast;
    } else if(c < -1*maxC){
      return -1*maxC + speedLast;
    } else {
      return speedNow;
    }
  }

  double state = 0;//This is for functions in the Teleopt... feel free to delete

	public void turnTo(double angle) {
    //Setting the Target Angle on the interval [0,360)
    angle %= 360;
    if(angle < 360 && angle >= 0){
      //Within Range
    } else if(angle > -360 && angle < 0){
      //Bellow Range
      angle += 360;
    } else {
      System.out.println("SOMETHINGS WRONG WITH TURNTO Target!");
    }

    //Getting the Current Angle and setting it on a postive interval of [0 , 360)
    double currentAng = gyro.getAngle()%360;
    if(currentAng < 360 && currentAng >= 0){
      //Within Postive Range
    } else if(currentAng > -360 && currentAng < 0){
      //Within Negative Range, need to get into positive range!
      currentAng += 360;
    } else {
      System.out.println("SOMETHINGS WRONG WITH TURNTO Current!");
    }

    //Now finding how far away we are and getting it on a range of (-180 , 180)
    double error = (currentAng - angle);
    if(error < 180 && error > -180){
      //Within range
    } else if(error <= -180 && error > -360){
      error += 360;
    } else if(error >= 180 && error < 360){
      error -= 360;
    } else {
      System.out.println("SOMETHINGS WRONG WITH TURNTO Error!");
    }
    
    //Now to set the speed at which we wish to turn
    double turnSpeed = error/12.5;
    if(Math.abs(error)<2){
      turnSpeed = 0;
    } else if(turnSpeed > 1){
      turnSpeed = 1;
    } else if(turnSpeed < -1){
      turnSpeed = -1;
    } else {
      //Between -1 and 1
    }
    
    //Telling Robot To Turn
    drive.tankDrive(-1*turnSpeed, turnSpeed);
    
    //TurnTo Diagnostics
    //System.out.println("Current Angle: " + currentAng + "\nSpeed We Are Turning: " + turnSpeed + "\nError Value " + error + "\nAngle we are turning to: " + angle);//Checks what the function is receiving and sending out
  }

  //The Controls Used During Sandstrom (Blind Teleope) and Teleope
  public void driveControls(String mode /*Only use for when locking things early game */){
    if(driveController.getStartButtonPressed()){
      compressOn = !compressOn;//Turns Compressor On and Off
    }
    compress.setClosedLoopControl(compressOn);

    if(!driveController.getYButton() || rampLock){
      double leftSpeed = driveController.getY(GenericHID.Hand.kRight) - driveController.getTriggerAxis(GenericHID.Hand.kLeft);
      double rightSpeed = driveController.getY(GenericHID.Hand.kLeft) - driveController.getTriggerAxis(GenericHID.Hand.kRight);
      if(driveController.getBumper(GenericHID.Hand.kLeft)){
        leftSpeed += 1;
      }
      if(driveController.getBumper(GenericHID.Hand.kRight)){
        rightSpeed += 1;
      }
      leftSpeed = accelCont(leftSpeed, lastSpeedL);
      rightSpeed = accelCont(rightSpeed,lastSpeedR);
      drive.tankDrive(leftSpeed, rightSpeed);
      lastSpeedL = leftSpeed;
      lastSpeedR = rightSpeed;
    } else if(driveController.getYButton()) {
      if(driveController.getPOV()==0){
        state = 0;
      } else if(driveController.getPOV()==45){
        state = 45;
      } else if(driveController.getPOV()==90){
        state = 90;
      } else if(driveController.getPOV()==135){
        state = 135;
      } else if(driveController.getPOV()==180){
        state = 180;
      } else if(driveController.getPOV()==225){
        state = 225;
      } else if(driveController.getPOV()==270){
        state = 270;
      } else if(driveController.getPOV()==315){
        state = 315;
      } else {
        //Keep state the same
      }
      turnTo(state);
    }

    //Lock the Ramp Operation
    if(driveController.getBackButtonPressed()){
      rampLock = !rampLock;
    }

    //Solinoids Hatch Control
    if(driveController.getBButton() || !rampLock){//Rotating the hatch-thing outward
      hatchRotateIn.set(false);
      hatchRotateOut.set(true);
    } else if((!rampOut && rampT == 0) && rampLock){
      hatchRotateIn.set(true);
      hatchRotateOut.set(false);
    }

    if(driveController.getXButton()){
      hatchEjectIn.set(false);
      hatchEjectOut.set(true);
    } else {
      hatchEjectIn.set(true);
      hatchEjectOut.set(false);
    }

    //Ramp Operations
    if(rampLock || driveController.getYButton()){
      rampC.set(0);
      driveController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    } else {
      if(driveController.getPOV()==180){
        rampC.set(0.5);
      } else if(driveController.getPOV()==0){
        rampC.set(-0.5);
      } else {
        rampC.set(0);
      }
      driveController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
    }
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //Pneumatics
    compress = new Compressor(0);

    hatchEjectOut = new Solenoid(hEO);
    hatchEjectIn = new Solenoid(hEI);
    hatchRotateIn = new Solenoid(hRI);
    hatchRotateOut = new Solenoid(hRO);

    //Sensors
    CameraServer.getInstance().startAutomaticCapture();
    CameraServer.getInstance().startAutomaticCapture();

    //Sensors
    gyro = new ADXRS450_Gyro(Port.kOnboardCS0);

    //Motors and Drive Controls
    frontLeft = new Spark(frontLeftChannel);
		backLeft = new Spark(backLeftChannel);
		frontRight = new Spark(frontRightChannel);
    backRight = new Spark(backRightChannel);
    
    ramp = new Spark(rampChannel);
    rampC = new SpeedControllerGroup(ramp);
			
		rightSide = new SpeedControllerGroup(frontLeft, backLeft);
    leftSide = new SpeedControllerGroup(frontRight, backRight);
    leftSide.setInverted(false);
    rightSide.setInverted(false);
    drive = new DifferentialDrive(leftSide, rightSide);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {//This year it operates like blind teleope
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {//This year it operates like blind teleope
    driveControls("auto");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    driveControls("tele");
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
