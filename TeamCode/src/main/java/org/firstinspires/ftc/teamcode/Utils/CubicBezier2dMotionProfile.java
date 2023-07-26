package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.geometry.Vector2d;

public class CubicBezier2dMotionProfile {

    private final Vector2d a, b, c;
    private double time = 1;

    public CubicBezier2dMotionProfile(Vector2d a, Vector2d b, Vector2d c, double time){
        this.a = a;
        this.b = b;
        this.c = c;
        this.time = time;
    }

    public Vector2d interpolate(double time){
        double k = time/this.time;
        k = Math.max(0,k);
        k = Math.min(1,k);

        Vector2d i1 = new Vector2d(k * (b.getX() - a.getX()) + a.getX(), k * (b.getY() - a.getY()) + a.getY());
        Vector2d i2 = new Vector2d(k * (c.getX() - b.getX()) + b.getX(), k * (c.getY() - b.getY()) + b.getY());

        return new Vector2d(k * (i2.getX() - i1.getX()) + i1.getX(), k * (i2.getY() - i1.getY()) + i1.getY());
    }
}
