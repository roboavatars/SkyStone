package org.firstinspires.ftc.teamcode.Splines;

public class SplineGenerator {
    public SplineGenerator(){

    }
    public Spline[] SplineBetweenTwoPoints(double startx, double starty, double endx,
                                           double endy, double starttheta, double endtheta,
                                           double startv, double endv, double starta, double enda,
                                           double startw, double endw, double time){
        double startxdot = startv*Math.cos(starttheta);
        double startydot = startv*Math.sin(starttheta);
        double endxdot = endv*Math.cos(endtheta);
        double endydot = endv*Math.sin(endtheta);
        double startxdotdot = starta*Math.cos(starttheta) - startv*Math.sin(starttheta)*startw;
        double startydotdot = starta*Math.sin(starttheta) + startv*Math.cos(starttheta)*startw;
        double endxdotdot = enda*Math.cos(endtheta) - endv*Math.sin(endtheta)*endw;
        double endydotdot = enda*Math.sin(endtheta) + endv*Math.cos(endtheta)*endw;

        Spline xspline = new Spline(startx,startxdot,startxdotdot,endx,endxdot,endxdotdot,time);
        Spline yspline = new Spline(starty,startydot,startydotdot,endy,endydot,endydotdot,time);
        Spline[] splines = {xspline,yspline};
        return splines;

    }
}
