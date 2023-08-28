package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class Follower implements IRobotModule {


    public static double followingCoefficient = 1, correctionCoefficient = 1, centripetalCorrectionCoefficient = 1, headingPIDCoefficient = 1;
    public static int segmentsPerCm = 1000;

    private final MecanumDrive drive;
    private final Localizer localizer;

    private Trajectory trajectory;
    private double currentFollowedPoint = 0;

    private final PIDController headingPIDController = new PIDController(0,0,0);

    public Follower(MecanumDrive drive, Localizer localizer){
        this.drive = drive;
        this.localizer = localizer;
    }

    public void setTrajectory(Trajectory newTrajectory){
        this.trajectory = newTrajectory;
        currentFollowedPoint = 0;
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }

    @Override
    public void update() {
        if(trajectory == null) return;

        Pose currentPose = localizer.getPoseEstimate();

        currentFollowedPoint = trajectory.getFollowedPoint(currentPose, currentFollowedPoint);

        Vector tangentVelocityVector = trajectory.getTangentVelocity(currentFollowedPoint).scaledBy(followingCoefficient);

        Pose trajectoryPose = trajectory.getPose(currentFollowedPoint);

        Vector correctingVector = new Vector(trajectoryPose.getX() - currentPose.getX(), trajectoryPose.getY() - currentPose.getY()).scaledBy(correctionCoefficient);

        Vector centripetalCorrectionVector = new Vector(Math.cos(Math.atan2(tangentVelocityVector.getY(), tangentVelocityVector.getX()) + PI/2.0),
                Math.sin(Math.atan2(tangentVelocityVector.getY(), tangentVelocityVector.getX()) + PI/2.0))
                .scaledBy(trajectory.getCurvature(currentFollowedPoint * centripetalCorrectionCoefficient));

        headingPIDController.setPID(MecanumDrive.headingPID.p, MecanumDrive.headingPID.i, MecanumDrive.headingPID.d);
        double headingDelta = trajectoryPose.getHeading() - currentPose.getHeading();
        headingDelta%=2.0 * PI;
        if(headingDelta > PI) headingDelta -= 2.0*PI;
        if(headingDelta < -PI) headingDelta += 2.0*PI;

        Vector turningVector = new Vector(0, 0, headingPIDController.calculate(0 , headingDelta)).scaledBy(headingPIDCoefficient);

        Vector driveVector = tangentVelocityVector.plus(correctingVector).plus(centripetalCorrectionVector).plus(turningVector);

        drive.setTargetVector(driveVector);
    }

    @Override
    public void emergencyStop() {

    }
}
