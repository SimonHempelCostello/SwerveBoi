/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
  private double upVal;
  private double downVal;
  private double upRevVal;
  private double downRevVal;
  private double upValDif;
  private double downValDif;
  private double upRevDif;
  private double downRevDif;
  private double lastThrottleDirection;
  private boolean shouldReverse;
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
    inputPos = (inputVector.getVectorAngleToi())/(2*Math.PI)-0.5;
    shouldReverse = false;

    upVal = (Math.ceil(currVal)+inputPos);
    upValDif = Math.abs(currVal-upVal);

    downVal = (Math.floor(currVal)+inputPos);
    downValDif = Math.abs(currVal-downVal);

    upRevVal = upVal + 0.5;
    upRevDif = Math.abs(currVal - upRevVal);

    downRevVal = downVal+0.5;
    downRevDif = Math.abs(currVal-downRevVal);
    System.out.println("upRevDif" + upRevDif);
    System.out.println("downDif" + downRevDif);
    System.out.println("upDif"+upValDif);
    System.out.println("downRevDif" + downValDif);
    if(upValDif<downValDif){
      if(upValDif>=downRevDif){
        outputPos = downRevVal;
        shouldReverse = true;
        
        modOffset = 0.5;
        SmartDashboard.putString("valString", "upRevVal");

      }
      else{
        shouldReverse = false;
        modOffset = 0;
        outputPos = upVal;
        SmartDashboard.putString("valString", "upVal");
      }
    }
    else{
      if(downValDif>=upRevDif){
        outputPos = upRevVal;
        shouldReverse = true;
        modOffset = -0.5;
        SmartDashboard.putString("valString", "downRevVal");
      }
      else{
        shouldReverse = false;
        outputPos = downVal;
        modOffset = 0;
        SmartDashboard.putString("valString", "downVal");

      }
    }

    SmartDashboard.putNumber("upval", upVal);
    SmartDashboard.putNumber("upRevval", upRevVal);
    SmartDashboard.putNumber("downVal", downVal);
    SmartDashboard.putNumber("downRevVal", downRevVal);
    SmartDashboard.putNumber("currVall", currVal);
    SmartDashboard.putNumber("inputPos", inputPos);
    SmartDashboard.putNumber("outputPos", outputPos);
    SmartDashboard.putBoolean("shouldReverse", shouldReverse);
    setAngle(outputPos);
    //setThrottle(inputVector.magnitude()*0.25, shouldReverse);
    
    currVal = outputPos+modOffset;
  }
  public double getAbsEncoderValue(){
    return aEncoder.getPeriod()*-1000000;
  }
  public void setAngle(double desiredPosition){
    mpidController.setReference(desiredPosition, ControlType.kPosition);
  }
  public void setThrottle(double desiredThrottle, boolean reverse){
    lastThrottleDirection = Math.signum(dMotor.get());
    if(lastThrottleDirection!=0){
      desiredThrottle = desiredThrottle*lastThrottleDirection;
      if(reverse){
        desiredThrottle = desiredThrottle*-1;
      }   
    }
    dMotor.set(desiredThrottle);
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
