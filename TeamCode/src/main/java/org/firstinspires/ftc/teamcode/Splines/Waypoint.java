package org.firstinspires.ftc.teamcode.Splines;

public class Waypoint {

    public double x;
    public double y;
    public double theta;
    public double velocity;
    public double acceleration;
    public double time;
    public Waypoint(double x, double y, double theta,
                    double velocity, double acceleration, double time){
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.time = time;
    }
}
