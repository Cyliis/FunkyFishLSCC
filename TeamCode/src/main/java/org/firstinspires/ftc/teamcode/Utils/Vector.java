package org.firstinspires.ftc.teamcode.Utils;

public class Vector {
    private final double x,y,z;

    public Vector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector(double x, double y){
        this(x,y,0);
    }

    public Vector(){
        this(0,0,0);
    }

    public static Vector fromAngleAndMagnitude(double t, double m){
        return new Vector(m*Math.cos(t), m*Math.sin(t));
    }

    //t1 is angle in xy plane, t2 is angle with xy plane
    public static Vector fromAngleAndMagnitude(double t1, double t2, double m){
        return new Vector(Math.cos(t1)*Math.cos(t2)*m, Math.sin(t1)*Math.cos(t2)*m, Math.sin(t2)*m);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public Vector plus(Vector other){
        return new Vector(x + other.getX(), y + other.getY(), z + other.getY());
    }

    public Vector scale(double a){
        return new Vector(x * a, y * a, z * a);
    }
}
