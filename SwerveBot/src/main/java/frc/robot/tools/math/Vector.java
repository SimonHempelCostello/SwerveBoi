package frc.robot.tools.math;
import java.lang.Math;

public class Vector {

   private double xVec;
   private double yVec;
   private double lastAngle;
   public Vector() {
	  xVec =0;
	  yVec = 0.0;
   }
   public Vector( boolean makeUnitVector, double x, double y){
      double magnitude = Math.sqrt(Math.pow(x,2)+Math.pow(y, 2));

      if(makeUnitVector){
         xVec = x/(magnitude);
         yVec = y/(magnitude);
      }
   }
   public Vector( double x, double y ) {
	  xVec = x;
	  yVec= y;
   }
   public double dot( Vector v1 ) {
	  return xVec*v1.getxVec() + yVec*v1.getyVec();
   }
   public double magnitude() {
	  return Math.sqrt ( xVec*xVec + yVec*yVec );
   }
   public double getxVec(){
	  return xVec;
   }
   public double getyVec(){
	  return yVec;
   }
   public void setX(double x){
	  xVec = x;
   }
   public void setY(double y){
	  yVec = y;
   }
   public double getVectorAngleToi(){
      if(Math.abs(xVec)<0.05 && Math.abs(yVec)<0.05){
         return lastAngle;
      }
      else{
         double angle;
         angle = Math.atan(yVec/xVec);
         if(xVec<0){
            angle = angle + Math.PI;
         }
         else if(xVec>=0 &&yVec<0){
            angle = angle + 2*Math.PI;
         }
         lastAngle = angle;
         return angle;
      }
   }
   public Vector scaleVector(double scalar){
      Vector resultantVector;
      try{
         resultantVector = new Vector(getxVec()*scalar, getyVec()*scalar);
      }
      catch(Exception E){
         resultantVector = new Vector(0, 0);
      }
    
      return resultantVector;
   }

   public Vector addVector( Vector vector2){
      Vector resultantVector = new Vector(getxVec()+vector2.getxVec(), getyVec() + vector2.getyVec());
      return resultantVector;
   }
} 