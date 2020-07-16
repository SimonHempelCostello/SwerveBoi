package frc.robot;
public class ButtonMap{
    private OI oi = new OI();
    public ButtonMap(){

    }
    public double driveStickUP(){
        return -oi.driverController.getRawAxis(1);
    }
    public double driveStickSideways(){
        return oi.driverController.getRawAxis(0);
    }
}