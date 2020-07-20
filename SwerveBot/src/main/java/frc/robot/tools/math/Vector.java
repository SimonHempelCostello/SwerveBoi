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
      if(Math.abs(xVec)<0.8 && Math.abs(yVec)<0.8){
         return lastAngle;
      }
      else{
         double angle;
         angle = Math.atan(yVec/xVec);
         if(xVec<0){
            angle = angle+Math.PI;
         }
         else if(xVec>0&&yVec<0){
            angle = 2*Math.PI+angle;
         }
         lastAngle = angle;
         if(angle<0){
            angle = 4.7123;
         }
         return angle;
      }
      

   }
   public Vector normalizeAddVector(Vector vector1, Vector vector2){
      Vector resultantVector = new Vector(0, 0);
      return resultantVector;
   }
} 