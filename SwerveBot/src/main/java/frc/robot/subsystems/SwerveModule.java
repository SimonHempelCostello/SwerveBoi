/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotStats;
import frc.robot.tools.math.Vector;


public class SwerveModule extends SubsystemBase {
  /**
   * Creates a new SwerveModule.
   */
  private double kP;
  private double kI;
  private double kD;
  private CANSparkMax tMotor;
  private CANSparkMax dMotor; 
  private CANPIDController mpidController;
  private Counter aEncoder;
  private double offset;
  private double startingAngle;
  private Vector outputVector= new Vector(0, 0);
  private double inputPos;
  private double currentPos;
  private double currentAbsPos;
  private double midDif;
  private double downDif;
  private double upDif;
  private List<Double> valList = new ArrayList<Double>(); 
  public SwerveModule(double P, double I, double D, CANSparkMax turnMotor, CANSparkMax driveMotor, Counter absoluteEncoder, double absEncoderOffset ) {
    kP = P;
    kI = I;
    kD = D;
    tMotor = turnMotor;
    dMotor = driveMotor;
    aEncoder = absoluteEncoder;
    offset = absEncoderOffset;
  }
  public void initModule(){
    mpidController = new CANPIDController(dMotor);
    tMotor.enableVoltageCompensation(11.3);
    mpidController = tMotor.getPIDController();
    mpidController.setOutputRange(-1, 1);
    mpidController.setIZone(0.1);
    mpidController.setP(kP);
    mpidController.setI(kI);
    mpidController.setD(kD);
    startingAngle = ((getAbsEncoderValue()+offset)/4096);
    SmartDashboard.putNumber("startingPosition", startingAngle);    
    tMotor.getEncoder().setPositionConversionFactor(RobotStats.moduleToEncoderRatio);
    tMotor.getEncoder().setPosition(startingAngle);
    //mpidController.setReference(Math.round(startingAngle),ControlType.kPosition );
    currentPos = tMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("setPos", 0);
  }
  public void setOutput(Vector inputVector){
    inputPos = (inputVector.getVectorAngleToi())/(2*Math.PI);
    currentAbsPos = currentPos%1;
    if(inputPos == currentAbsPos){
      upDif = 0;
      downDif = 0;
    }
    if(inputPos>currentAbsPos){
      upDif = Math.abs(inputPos-currentAbsPos);
      downDif = -Math.abs((inputPos-1)-currentAbsPos);    
    }
    else{
      upDif = Math.abs((inputPos+1)-currentAbsPos);
      downDif = -Math.abs(currentAbsPos-inputPos);
    }
    SmartDashboard.putNumber("inputPos", inputPos);
    SmartDashboard.putNumber("currentAbsPos", currentAbsPos);
    SmartDashboard.putNumber("A upDif", upDif);
    SmartDashboard.putNumber("A downDif", downDif);
    SmartDashboard.putNumber("currentPos", currentPos);
    if(Math.abs(upDif)<Math.abs(downDif)){
      currentPos = currentPos+upDif;
    }
    else{
      currentPos = currentPos + downDif;
    }
  }
  public double getAbsEncoderValue(){
    return aEncoder.getPeriod()*-1000000;
  }
  public void setAngle(double desiredPosition){
    mpidController.setReference(desiredPosition, ControlType.kPosition);
  }
  public void setThrottle(double desiredThrottle, boolean reverse){
    if(reverse){
      dMotor.set(-desiredThrottle);
    }
    else{
      dMotor.set(desiredThrottle);
    }
  }
  public double getAngle(){
    return tMotor.getEncoder().getPosition();
  }
  public void teleopPeriodic(){
    outputVector.setX(RobotMap.buttonMap.driveStickSideways());
    outputVector.setY(RobotMap.buttonMap.driveStickUP());

    setOutput(outputVector);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
