package org.firstinspires.ftc.teamcode.TrajectoryStuff;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Math.Polynomial;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

import java.util.ArrayList;

@Config
public class CubicBezierTrajectorySegment extends TrajectorySegment{

    private final Pose startPoint, endPoint;
    private final Pose controlPoint0, controlPoint1;

    private Polynomial pX, pY, pH;
    private Polynomial pdX, pdY;
    private Polynomial pd2X, pd2Y;

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
        double xa = endPoint.getX() - 3.0 * controlPoint1.getX() + 3.0 * controlPoint0.getX() - startPoint.getX();
        double xb = 3.0 * (controlPoint1.getX() - 2.0 * controlPoint0.getX() + startPoint.getX());
        double xc = 3.0 * (controlPoint0.getX() - startPoint.getX());
        double xd = startPoint.getX();

        pX = new Polynomial(xa, xb, xc, xd);
        pdX = pX.getDerivative();
        pd2X = pdX.getDerivative();

        double ya = endPoint.getY() - 3.0 * controlPoint1.getY() + 3.0 * controlPoint0.getY() - startPoint.getY();
        double yb = 3.0 * (controlPoint1.getY() - 2.0 * controlPoint0.getY() + startPoint.getY());
        double yc = 3.0 * (controlPoint0.getY() - startPoint.getY());
        double yd = startPoint.getY();

        pY = new Polynomial(ya, yb, yc, yd);
        pdY = pY.getDerivative();
        pd2Y = pdY.getDerivative();

        double ha = endPoint.getHeading() - 3.0 * controlPoint1.getHeading() + 3.0 * controlPoint0.getHeading() - startPoint.getHeading();
        double hb = 3.0 * (controlPoint1.getHeading() - 2.0 * controlPoint0.getHeading() + startPoint.getHeading());
        double hc = 3.0 * (controlPoint0.getHeading() - startPoint.getHeading());
        double hd = startPoint.getHeading();

        pH = new Polynomial(ha, hb, hc, hd);

        computeLength();
    }

    private void computeLength(){
        double dx, dy;
        double dt = 1.0/(double)resolution;
        for(double d = 0; d <= 1; d += dt) {
            lengthArray.add(length);

            dx = pdX.evaluate(d);
            dy = pdY.evaluate(d);

            length+=Math.sqrt(dx*dx + dy*dy)*dt;
        }
    }

    @Override
    public Vector getTangentVelocity(double t) {
        Vector ans = new Vector(pdX.evaluate(t), pdY.evaluate(t)).scaledToMagnitude(1);
        return ans;
    }

    @Override
    public double getCurvature(double t) {
        double dx = pdX.evaluate(t);
        double dy = pdY.evaluate(t);
        double d2x = pd2X.evaluate(t);
        double d2y = pd2Y.evaluate(t);

        return (dx * d2y - d2x * dy) /
                (Math.sqrt(Math.pow(dx * dx  + dy * dy,3)));
    }

    @Override
    public Pose getPose(double t) {
        return new Pose(pX.evaluate(t), pY.evaluate(t), pH.evaluate(t));
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
        int index = (int)(t * (double)resolution);
        index = Math.min(resolution - 1, Math.max(0,index));
        return lengthArray.get(index) + (lengthArray.get(Math.min(resolution - 1, index + 1)) - lengthArray.get(index)) * 0.5;
    }

    @Override
    public double getLength(){
        return length;
    }


}
