
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
}