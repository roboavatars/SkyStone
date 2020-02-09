package org.firstinspires.ftc.teamcode.Splines;

import java.util.ArrayList;
import android.util.Log;

public class Path {
    private ArrayList<Spline[]> splines = new ArrayList<Spline[]>();
    private ArrayList<Double> waypointTimes = new ArrayList<Double>();
    private ArrayList<Waypoint> waypoints;
    private double totaltime;

    public Path(ArrayList<Waypoint> waypoints){
        //defining waypoint arraylist
        this.waypoints = waypoints;

        //define splinegenerate all the splines necessary
        SplineGenerator splinegen = new SplineGenerator();

        //find total time to make sure nothing is going wrong on Pose calls
        totaltime = waypoints.get(waypoints.size()-1).time;

        for (int i = 0; i < waypoints.size()-1; i++) {
            //getting the relevant waypoints
            Waypoint waypoint1 = waypoints.get(i);
            Waypoint waypoint2 = waypoints.get(i+1);

            //finding all the variables for the spline
            double startx = waypoint1.x;
            double starty = waypoint1.y;
            double endx = waypoint2.x;
            double endy = waypoint2.y;
            double starttheta = waypoint1.theta;
            double endtheta = waypoint2.theta;
            double startv = waypoint1.velocity;
            double endv = waypoint2.velocity;
            double starta = waypoint1.acceleration;
            double enda = waypoint2.acceleration;
            double startw = 0;
            double endw = 0;
            double time = waypoint2.time-waypoint1.time;

            //making sure waypoints are correct
            assert waypoint2.time>waypoint1.time: "Waypoint times are not correct";
            assert waypoint1.velocity==0 || waypoint2.velocity==0: "Waypoint Velocity is zero";

            //generating splines and adding them to the array
            Spline[] segment = splinegen.SplineBetweenTwoPoints(startx, starty, endx,
                    endy, starttheta, endtheta, startv, endv, starta, enda, startw, endw, time);
            splines.add(segment);

            //adding the time
            waypointTimes.add(waypoint2.time);
        }
    }
    public Pose getRobotPose(double time){

        int splineindex = 0;
        if(totaltime<=time){
            splineindex = waypointTimes.size()-1;
        }
        else{
            for (int i = 0; i < waypointTimes.size(); i++) {
                if(waypointTimes.get(i)<time){
                    splineindex = i;
                }
                else{
                    break;
                }
            }
        }
        double splinetime = time - waypoints.get(splineindex).time;
        Spline[] currentspline = splines.get(splineindex);
        double x = currentspline[0].position(splinetime);
        double y = currentspline[1].position(splinetime);
        double theta = Math.atan2(currentspline[1].velocity(splinetime),
                currentspline[0].velocity(splinetime));
        return new Pose(x, y, theta);
    }
}
