package frc.robot;
public class ButtonMap{
    private OI oi = new OI();
    public ButtonMap(){

    }
    public double driveStickUP(){
        return -oi.driverController.getRawAxis(1);
    }
    public double driveStickSideways(){
        return -oi.driverController.getRawAxis(0);
    }
    public double turnStickUP(){
        return -oi.driverController.getRawAxis(5);
    }
    public double turnStickSideways(){
        return -oi.driverController.getRawAxis(4);
    }
}