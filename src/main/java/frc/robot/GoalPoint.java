
package frc.robot;

public class GoalPoint{
    private double Xpos;
    private double Ypos;
    private boolean WillShoot;
    private boolean WillIntake;

    public GoalPoint(double Xpos, double Ypos, boolean WillShoot, boolean WillIntake){
        this.Xpos = Xpos;
        this.Ypos = Ypos;
        this.WillShoot = WillShoot;
        this.WillIntake = WillIntake;
    }
    
    public double getX()
    {
        return Xpos;
    }

      public double getY()
    {
        return Ypos;
    }

      public boolean GetWillShoot()
    {
        return WillShoot;
    }

      public boolean getWillIntake()
    {
        return WillIntake;
    }
}   