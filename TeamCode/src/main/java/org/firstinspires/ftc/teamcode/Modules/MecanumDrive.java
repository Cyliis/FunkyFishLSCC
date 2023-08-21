package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Utils.CoolMotor;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Trajectory;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class MecanumDrive implements IRobotModule {

    private final Localizer localizer;

    private final CoolMotor frontLeft, frontRight, backLeft, backRight;
    public static String frontLeftMotorName = "fl", frontRightMotorName = "fr",
    backLeftMotorName = "bl", backRightMotorName = "br";
    public static boolean frontLeftMotorReversed = false, frontRightMotorReversed = true, backLeftMotorReversed = false, backRightMotorReversed = true;

    public static PIDCoefficients translationalPID, headingPID;
    private final PIDController tpid= new PIDController(0,0,0), hpid = new PIDController(0,0,0);

    public static double lateralMultiplier = Math.sqrt(2);

    public enum RunMode{
        PID, VECTOR
    }

    private RunMode runMode;

    public MecanumDrive(HardwareMap hm, Localizer localizer, RunMode runMode){
        this.localizer = localizer;
        frontLeft = new CoolMotor(hm, frontLeftMotorName, CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(hm, frontRightMotorName, CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(hm, backLeftMotorName, CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(hm, backRightMotorName, CoolMotor.RunMode.RUN, backRightMotorReversed);
        this.runMode = runMode;
    }

    public MecanumDrive(HardwareMap hm, Localizer localizer){
        this(hm, localizer, RunMode.VECTOR);
    }

    private Vector powerVector = new Vector();
    private Pose targetPose = new Pose();
    private Vector targetVector = new Vector();

    public void setTargetPose(Pose pose){
        this.targetPose = pose;
    }

    public void setTargetVector(Vector vector){
        this.targetVector = vector;
    }

    public RunMode getRunMode() {
        return runMode;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    private void updatePowerVector(){
        switch (runMode){
            case VECTOR:
                powerVector = new Vector(targetVector.getX(), targetVector.getY() * lateralMultiplier * targetVector.getZ());
                break;
            case PID:
                Pose currentPose = localizer.getPose();

                double xDiff = targetPose.getX() - currentPose.getX();
                double yDiff = targetPose.getY() - currentPose.getY();

                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                tpid.setPID(translationalPID.p, translationalPID.i, translationalPID.d);

                double translationalPower = tpid.calculate(0, distance);

                powerVector = new Vector(translationalPower * Math.cos(Math.atan2(yDiff, xDiff)), translationalPower * Math.sin(Math.atan2(yDiff, xDiff)), 0);

                double headingDiff = targetPose.getHeading() - currentPose.getHeading();

                while(headingDiff > PI) headingDiff -= PI;
                while(headingDiff < -PI) headingDiff += PI;

                hpid.setPID(headingPID.p, headingPID.i, headingPID.d);

                double headingPower = hpid.calculate(0, headingDiff);

                powerVector.plus(new Vector(0,0,headingPower));
                break;
        }
        powerVector.scale(Math.max(1, Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ())));
    }

    private void updateMotors(){
        frontLeft.setPower(powerVector.getX() - powerVector.getY() - powerVector.getZ());
        frontRight.setPower(powerVector.getX() + powerVector.getY() + powerVector.getZ());
        backLeft.setPower(powerVector.getX() + powerVector.getY() - powerVector.getZ());
        backRight.setPower(powerVector.getX() - powerVector.getY() + powerVector.getZ());

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    @Override
    public void update() {
        updatePowerVector();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        powerVector = new Vector();
        updateMotors();
    }
}
