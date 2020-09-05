/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.sensors.Navx;
import frc.robot.tools.math.Vector;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private static double kP = 1.8;
  private static double kI = 0.02;
  private static double kD = 18;
  private SwerveModule backLeftSwerveModule = new SwerveModule( kP, kI, kD, RobotMap.backLeftTurn, RobotMap.backLeftDrive, RobotMap.backLeftAbsEnconder, 524.55);
  private SwerveModule backRightSwerveModule = new SwerveModule( kP, kI, kD,RobotMap.backRightTurn, RobotMap.backRightDrive, RobotMap.backRightAbsEnconder, 2039.7);
  private SwerveModule frontLeftSwerveModule = new SwerveModule( kP, kI, kD, RobotMap.frontLeftTurn, RobotMap.frontLeftDrive, RobotMap.frontLeftAbsEnconder, 1036.55);
  private SwerveModule frontRightSwerveModule = new SwerveModule( kP, kI, kD, RobotMap.frontRightTurn, RobotMap.frontRightDrive, RobotMap.frontRightAbsEnconder,3596.7);
  private Vector directionInputVector = new Vector(0, 0);
  private Vector turnInputVector = new Vector(0,0);
  private Vector backLeftOutputVector = new Vector(0,0);
  private Vector backRightOutputVector = new Vector(0,0);
  private Vector frontLeftOutputVector = new Vector(0,0);
  private Vector frontRightOutputVector = new Vector(0,0);
  private double inverseSRT2 = 1/Math.sqrt(2);
  private double turnNumber;
  private double desiredAngle;
  public Drivetrain() {

  }
  public void initializeDriveTrain(){
    backLeftSwerveModule.initModule();
    backRightSwerveModule.initModule();
    frontLeftSwerveModule.initModule();
    frontRightSwerveModule.initModule();
  }
  public void teleopPeriodic() {
    if(Math.abs(RobotMap.buttonMap.driveStickUP())>0.1||Math.abs(RobotMap.buttonMap.driveStickSideways())>0.05){
      directionInputVector.setX(RobotMap.buttonMap.driveStickUP());
      directionInputVector.setY(RobotMap.buttonMap.driveStickSideways());
    }
    else{
      directionInputVector.setX(0);
      directionInputVector.setY(0);
    }
   
    turnNumber = RobotMap.buttonMap.turnStickSideways();
    if(Math.abs(turnNumber)>0.1){
      if(Math.signum(turnNumber)==-1){
        backLeftOutputVector.setX(-inverseSRT2);
        backLeftOutputVector.setY(-inverseSRT2);

        backRightOutputVector.setX(inverseSRT2);
        backRightOutputVector.setY(-inverseSRT2);

        frontRightOutputVector.setX(inverseSRT2);
        frontRightOutputVector.setY(inverseSRT2);

        frontLeftOutputVector.setX(-inverseSRT2);
        frontLeftOutputVector.setY(inverseSRT2);
      }
      else{
        backLeftOutputVector.setX(inverseSRT2);
        backLeftOutputVector.setY(inverseSRT2);

        backRightOutputVector.setX(-inverseSRT2);
        backRightOutputVector.setY(inverseSRT2);

        frontRightOutputVector.setX(-inverseSRT2);
        frontRightOutputVector.setY(-inverseSRT2);

        frontLeftOutputVector.setX(inverseSRT2);
        frontLeftOutputVector.setY(-inverseSRT2);

      }
    }
    else{
        backLeftOutputVector.setX(0);
        backLeftOutputVector.setY(0);

        backRightOutputVector.setX(0);
        backRightOutputVector.setY(0);

        frontRightOutputVector.setX(0);
        frontRightOutputVector.setY(0);
        
        frontLeftOutputVector.setX(0);
        frontLeftOutputVector.setY(0);
    }
    backLeftOutputVector = backLeftOutputVector.addVector(directionInputVector);
    backLeftOutputVector.scaleVector(1/backLeftOutputVector.magnitude());
    backRightOutputVector = backRightOutputVector.addVector(directionInputVector);
    backRightOutputVector.scaleVector(1/backRightOutputVector.magnitude());

    frontLeftOutputVector = frontLeftOutputVector.addVector(directionInputVector);
    frontLeftOutputVector.scaleVector(1/frontLeftOutputVector.magnitude());

    frontRightOutputVector = frontRightOutputVector.addVector(directionInputVector);
    frontRightOutputVector.scaleVector(1/frontRightOutputVector.magnitude());



    backLeftSwerveModule.setOutput(backLeftOutputVector);
    backRightSwerveModule.setOutput(backRightOutputVector);
    frontLeftSwerveModule.setOutput(frontLeftOutputVector);
    frontRightSwerveModule.setOutput(frontRightOutputVector);


    // This method will be called once per scheduler run
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
