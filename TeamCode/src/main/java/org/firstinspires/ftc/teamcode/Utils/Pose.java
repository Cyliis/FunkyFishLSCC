package org.firstinspires.ftc.teamcode.Utils;

public class Pose {
    private Vector2d position;
    private double heading;

    public Pose(Vector2d position, double heading){
        this.position = position;
        this.heading = heading;
    }

    public Vector2d getPosition() {
        return position;
    }

    public double getHeading() {
        return heading;
    }

    public void setPosition(Vector2d position) {
        this.position = position;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }
}
