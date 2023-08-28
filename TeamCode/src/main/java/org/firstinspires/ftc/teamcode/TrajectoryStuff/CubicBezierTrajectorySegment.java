package org.firstinspires.ftc.teamcode.TrajectoryStuff;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

import java.util.ArrayList;

@Config
public class CubicBezierTrajectorySegment extends TrajectorySegment{

    private final Pose startPoint, endPoint;
    private final Pose controlPoint0, controlPoint1;

    private double xa = 0, xb = 0, xc = 0, xd = 0;
    private double ya = 0, yb = 0, yc = 0, yd = 0;
    private double ha = 0, hb = 0, hc = 0, hd = 0;

    private double dxa = 0, dxb = 0, dxc = 0;
    private double dya = 0, dyb = 0, dyc = 0;

    private double d2xa = 0, d2xb = 0;
    private double d2ya = 0, d2yb = 0;

    private double length = 0;

    private ArrayList<Double> lengthArray = new ArrayList<>();

    public CubicBezierTrajectorySegment(Pose startPoint, Pose controlPoint0, Pose controlPoint1, Pose endPoint){
        this.startPoint = startPoint;
        this.controlPoint0 = controlPoint0;
        this.controlPoint1 = controlPoint1;
        this.endPoint = endPoint;
        init();
    }

    public CubicBezierTrajectorySegment(CubicBezierTrajectorySegment previousCurve, Pose controlPoint1, Pose endPoint){
        this.startPoint = previousCurve.endPoint;
        this.controlPoint0 = new Pose(previousCurve.endPoint.getX() + (previousCurve.endPoint.getX() - previousCurve.controlPoint1.getX()),
                previousCurve.endPoint.getY() + (previousCurve.endPoint.getY() - previousCurve.controlPoint1.getY()), previousCurve.controlPoint1.getHeading());
        this.controlPoint1 = controlPoint1;
        this.endPoint = endPoint;
        init();
    }

    private void init(){
        xa = endPoint.getX() - 3.0 * controlPoint1.getX() + 3.0 * controlPoint0.getX() - startPoint.getX();
        xb = 3.0 * (controlPoint1.getX() - 2.0 * controlPoint0.getX() + startPoint.getX());
        xc = 3.0 * (controlPoint0.getX() - startPoint.getX());
        xd = startPoint.getX();

        dxa = 3.0 * xa;
        dxb = 2.0 * xb;
        dxc = xc;

        d2xa = 2.0 * dxa;
        d2xb = dxb;

        ya = endPoint.getY() - 3.0 * controlPoint1.getY() + 3.0 * controlPoint0.getY() - startPoint.getY();
        yb = 3.0 * (controlPoint1.getY() - 2.0 * controlPoint0.getY() + startPoint.getY());
        yc = 3.0 * (controlPoint0.getY() - startPoint.getY());
        yd = startPoint.getY();

        dya = 3.0 * ya;
        dyb = 2.0 * yb;
        dyc = yc;

        d2ya = 2.0 * dya;
        d2yb = dyb;

        ha = endPoint.getHeading() - 3.0 * controlPoint1.getHeading() + 3.0 * controlPoint0.getHeading() - startPoint.getHeading();
        hb = 3.0 * (controlPoint1.getHeading() - 2.0 * controlPoint0.getHeading() + startPoint.getHeading());
        hc = 3.0 * (controlPoint0.getHeading() - startPoint.getHeading());
        hd = startPoint.getHeading();

        computeLength();
    }

    private void computeLength(){
        double dx, dy;
        double dt = 1.0/(double)resolution;
        for(double d = 0; d <= 1; d += dt) {
            lengthArray.add(length);

            dx = d*d*dxa + d*dxb + dxc;
            dy = d*d*dya + d*dyb + dyc;

            length+=Math.sqrt(dx*dx + dy*dy)*dt;
        }
    }

    @Override
    public Vector getTangentVelocity(double t) {
        Vector ans = new Vector(dxa * t*t + dxb * t + dxc, dya * t*t + dyb * t + dyc);
        ans.scaleToMagnitude(1);
        return ans;
    }

    @Override
    public double getCurvature(double t) {
        double dx = dxa * t*t + dxb + t + dxc;
        double dy = dya * t*t + dyb + t + dyc;
        double d2x = d2xa * t + d2xb;
        double d2y = d2ya * t + d2yb;

        return (dx * d2y - d2x * dy) /
                (Math.sqrt(Math.pow(dx * dx  + dy * dy,3)));
    }

    @Override
    public Pose getPose(double t) {
        return new Pose(xa * t*t*t + xb * t*t + xc * t + xd,
                ya * t*t*t + yb * t*t + yc * t + yd,
                ha * t*t*t + hb * t*t + hc * t + hd);
    }

    @Override
    public double getHeading(double t) {
        return getPose(t).getHeading();
    }

    @Override
    public double getDistanceFromPoint(Pose pose, double t) {
        return new Vector(pose.getX(), pose.getY()).plus(new Vector(-getPose(t).getX(), -getPose(t).getY())).getMagnitude();
    }

    @Override
    public double getLengthAt(double t) {
        int index = (int)(t * (1.0/(double)resolution));
        index = Math.min(resolution, Math.max(0,index));
        return lengthArray.get(index) + (lengthArray.get(Math.min(resolution, index + 1)) - lengthArray.get(index)) * (t - 1.0/(double)index);
    }

    @Override
    public double getLength(){
        return length;
    }


}
