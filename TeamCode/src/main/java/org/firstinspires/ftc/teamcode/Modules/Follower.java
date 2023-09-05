package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class Follower implements IRobotModule {


    public static double followingCoefficient = 1, correctionCoefficient = 1, centripetalCorrectionCoefficient = 1, headingPIDCoefficient = 1;
    public static int segmentsPerUnit = 100;

    private final MecanumDrive drive;
    private final Localizer localizer;

    private Trajectory trajectory;
    public double currentFollowedPoint = 0;

    private final PIDController headingPIDController = new PIDController(0,0,0);

    public static double defaultPIDThreshold = 2.5;

    private double PIDThreshold = defaultPIDThreshold;

    private boolean pid = false;

    public static PIDCoefficients correctionPIDCoefficients = new PIDCoefficients(0,0,0);
    private PIDController correctionPID = new PIDController(0,0,0);

    public Follower(MecanumDrive drive, Localizer localizer){
        this.drive = drive;
        this.localizer = localizer;
    }

    public void setTrajectory(Trajectory newTrajectory, double PIDThreshold){
        this.trajectory = newTrajectory;
        this.PIDThreshold = PIDThreshold;
        this.pid = false;
        this.drive.setRunMode(MecanumDrive.RunMode.Vector);
        this.currentFollowedPoint = 0;
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }

    @Override
    public void update() {
        if(trajectory == null) return;
        if(pid) return;

        if(trajectory.getLength() - trajectory.getLengthAt(currentFollowedPoint) <= PIDThreshold){
            pid = true;
            drive.setRunMode(MecanumDrive.RunMode.PID);
            drive.setTargetPose(trajectory.getPose(1));
        }

        Pose currentPose = localizer.getPoseEstimate();

        currentFollowedPoint = trajectory.getFollowedPoint(currentPose, currentFollowedPoint);

        Vector tangentVelocityVector = trajectory.getTangentVelocity(currentFollowedPoint).scaledBy(followingCoefficient);

        Pose trajectoryPose = trajectory.getPose(currentFollowedPoint);

        correctionPIDCoefficients = MecanumDrive.translationalPID;
        correctionPID.setPID(correctionPIDCoefficients.p, correctionPIDCoefficients.i, correctionPIDCoefficients.d);

        Vector correctingVector = new Vector(trajectoryPose.getX() - currentPose.getX(), trajectoryPose.getY() - currentPose.getY());
        double correctionPower = correctionPID.calculate(-correctingVector.getMagnitude(),0);
        correctingVector.scaledToMagnitude(correctionPower).scaledBy(correctionCoefficient);

        Vector centripetalCorrectionVector = new Vector(Math.cos(Math.atan2(tangentVelocityVector.getY(), tangentVelocityVector.getX()) + PI/2.0),
                Math.sin(Math.atan2(tangentVelocityVector.getY(), tangentVelocityVector.getX()) + PI/2.0))
                .scaledBy(trajectory.getCurvature(currentFollowedPoint * centripetalCorrectionCoefficient));

        headingPIDController.setPID(MecanumDrive.headingPID.p, MecanumDrive.headingPID.i, MecanumDrive.headingPID.d);
        double headingDelta = trajectoryPose.getHeading() - currentPose.getHeading();
        headingDelta%=2.0 * PI;
        if(headingDelta > PI) headingDelta -= 2.0*PI;
        if(headingDelta < -PI) headingDelta += 2.0*PI;

        Vector turningVector = new Vector(0, 0, headingPIDController.calculate(-headingDelta, 0)).scaledBy(headingPIDCoefficient);

        Vector driveVector = tangentVelocityVector.plus(correctingVector).plus(centripetalCorrectionVector).plus(turningVector);

        drive.setTargetVector(driveVector);
    }

    @Override
    public void emergencyStop() {

    }
}
