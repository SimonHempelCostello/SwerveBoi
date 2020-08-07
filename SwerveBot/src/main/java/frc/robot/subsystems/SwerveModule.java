/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

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
  private double currVal;
  private double inputPos;
  private double outputPos;
  private double upInput;
  private double downInput;
  private double upRevInput;
  private double downRevInput;
  private double upDif;
  private double downDif;
  private double upRevDif;
  private double downRevDif;
  private double currentAbsPosition;
  private double currentAbsAngle;
  private double inputPosAngle;
  private double lastThrottleDirection;
  private ArrayList<Double> difArrayList = new ArrayList<Double>();

  private double modOffset;
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
    mpidController.setReference(Math.round(startingAngle),ControlType.kPosition );
    SmartDashboard.putNumber("setPos", 0);
  }
  public void setOutput(Vector inputVector){
    inputPos = (inputVector.getVectorAngleToi())/(2*Math.PI);
    upInput = inputPos;
    downInput  = -inputPos;
    upRevInput = inputPos -0.5;
    downRevInput = -inputPos + 0.5;
    currentAbsPosition  = currVal%1;
    upDif = upInput-currentAbsPosition;
    downDif = downInput-currentAbsPosition;
    upRevDif = upRevInput - currentAbsPosition;
    downRevDif = downRevInput-currentAbsPosition;
    
    if(Math.abs(upDif)<Math.abs(downDif)&&Math.abs(upDif)<Math.abs(upRevDif)&&Math.abs(upDif)<Math.abs(downRevDif)){
      currVal = currVal + upDif;
    }
    else if(Math.abs(downDif)<Math.abs(upRevDif)&&Math.abs(downDif)<Math.abs(downRevDif)){
      currVal = currVal +downDif;
    }
    else if(Math.abs(upRevDif)<Math.abs(downRevDif)){
      currVal = currVal + upRevDif;
    }
    else{
      currVal = currVal + downRevDif;
    }
    /*SmartDashboard.putNumber("upInput", upInput);
    SmartDashboard.putNumber("downInput", downInput);
    SmartDashboard.putNumber("upRevInput", upRevInput);
    SmartDashboard.putNumber("downRevInput", downRevInput);
    SmartDashboard.putNumber("currVal", currentAbsPosition);
    SmartDashboard.putNumber("upDif", upDif);
    SmartDashboard.putNumber("downDif", downDif);*/
    currentAbsAngle = Math.abs((currVal%1)*2*Math.PI);
    inputPosAngle = Math.abs(inputPos*2*Math.PI);
    SmartDashboard.putBoolean("rev", Math.abs(currentAbsAngle-inputPosAngle)>=180);
    SmartDashboard.putNumber("currentAbleAngle",currentAbsAngle );
    SmartDashboard.putNumber("inputPosAngle", inputPosAngle);
    if(Math.abs(currentAbsAngle-inputPosAngle)>90){
      setThrottle(inputVector.magnitude()*0.25, true);
    }
    else{
      setThrottle(inputVector.magnitude()*0.25, false);

    }
   

    setAngle(currVal);
 
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
    outputVector.setX(-RobotMap.buttonMap.driveStickSideways());
    outputVector.setY(RobotMap.buttonMap.driveStickUP());

    setOutput(outputVector);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
