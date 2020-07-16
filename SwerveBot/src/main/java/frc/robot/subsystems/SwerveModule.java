/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
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
  private double kF;
  private double kP;
  private double kI;
  private double kD;
  private CANSparkMax tMotor;
  private CANSparkMax dMotor; 
  private CANPIDController mpidController;
  private CANEncoder moduleEncoder;
  private Counter aEncoder;
  private double offset;
  private double desiredPosision;
  private double startingAngle;
  private Vector outputVector= new Vector(0, 0);
  private double currVal;
  private double inputPos;
  private double outputPos;
  private double upVal;
  private double downVal;
  private double upRevVal;
  private double downRevVal;
  private double upValDif;
  private double downValDif;
  private double upRevDif;
  private double downRevDif;
  public SwerveModule(double F, double P, double I, double D, CANSparkMax turnMotor, CANSparkMax driveMotor, Counter absoluteEncoder, double absEncoderOffset ) {
    kF = F; 
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
    moduleEncoder = tMotor.getEncoder();
    mpidController.setOutputRange(-1, 1);
    mpidController.setIZone(0.1);
    mpidController.setP(kP);
    mpidController.setI(kI);
    mpidController.setD(kD);
    startingAngle = ((getAbsEncoderValue()+offset)/4096);
    SmartDashboard.putNumber("startingPosition", startingAngle);    tMotor.getEncoder().setPositionConversionFactor(0.05555);
    tMotor.getEncoder().setPosition(startingAngle);
    mpidController.setReference(Math.round(startingAngle),ControlType.kPosition );
    SmartDashboard.putNumber("setPos", 0);

  }
  public void setOutput(double desiredAngle){
    inputPos = (desiredAngle)/(2*Math.PI)-0.5;

    upVal = (Math.ceil(currVal)+inputPos);
    upValDif = Math.abs(currVal-upVal);
    downVal = (Math.floor(currVal)+inputPos);
    downValDif = Math.abs(currVal-downVal);
    upRevVal = upVal-0.5;
    upRevDif = Math.abs(currVal - upRevVal);
    downRevVal = downVal-0.5;
    downRevDif = Math.abs(currVal-downRevVal);

    if(upValDif<downValDif){
      outputPos = upVal;
      System.out.println("upVal");
    }
    else{
      System.out.println("downVal");

      outputPos = downVal;
    }
    SmartDashboard.putNumber("upval", upVal);
    SmartDashboard.putNumber("downVal", downVal);
    

    SmartDashboard.putNumber("currVall", currVal);
    SmartDashboard.putNumber("inputPos", inputPos);
    SmartDashboard.putNumber("outputPos", outputPos);
    
    
    mpidController.setReference(outputPos, ControlType.kPosition);
    currVal = outputPos;
  }
  public double getAbsEncoderValue(){
    return aEncoder.getPeriod()*-1000000;
  }
  public double getAngle(){
    return tMotor.getEncoder().getPosition();
  }
  public void teleopPeriodic(){
    outputVector.setX(RobotMap.buttonMap.driveStickSideways());
    outputVector.setY(RobotMap.buttonMap.driveStickUP());

    setOutput(outputVector.getVectorAngleToi());

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
