package org.firstinspires.ftc.teamcode.Utils;

public class Vector2d {
    private final double x,y;

    public Vector2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static Vector2d fromMagnitudeAndAngle(double m, double theta){
        return new Vector2d(m * Math.cos(theta), m * Math.sin(theta));
    }

    public static Vector2d unitVector(double theta){
        return new Vector2d(Math.cos(theta), Math.sin(theta));
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getAngle(){
        return Math.atan2(y,x);
    }

    public double getMagnitude(){
        return Math.sqrt(x*x + y*y);
    }
}
