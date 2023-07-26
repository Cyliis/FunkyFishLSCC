package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Utils.CoolMotor;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Vector2d;

@Config
public class DriveTrain implements IRobotModule {

    private CoolMotor left1, left2, right1, right2;
    public static String Left1MotorName = "left1", Left2MotorName = "left2",
    Right1MotorName = "right1", Right2MotorName = "right2";
    public static boolean left1Reversed = false, left2Reversed = false, right1Reversed = false, right2Reversed = false;

    Localizer localizer;

    private Vector2d driveVector;
    private double leftPower, rightPower;

    public enum DriveMode{
        ByValues, ByVector;
    }

    public static PIDCoefficients rotationPIDCoefficients = new PIDCoefficients(0,0,0);
    private PIDController rotationPid = new PIDController(0,0,0);

    private DriveMode driveMode;

    public DriveTrain(HardwareMap hm, DriveMode driveMode){
        left1 = new CoolMotor(hm, Left1MotorName, CoolMotor.RunMode.RUN, left1Reversed);
        left2 = new CoolMotor(hm, Left2MotorName, CoolMotor.RunMode.RUN, left2Reversed);
        right1 = new CoolMotor(hm, Right1MotorName, CoolMotor.RunMode.RUN, right1Reversed);
        right2 = new CoolMotor(hm, Right2MotorName, CoolMotor.RunMode.RUN, right2Reversed);
        
        localizer = new Localizer(hm);

        driveMode = DriveMode.ByValues;
    }

    public void setDriveValues(double forward, double rotation){
        if(driveMode != DriveMode.ByValues) return;
        
        rightPower = forward + rotation;
        leftPower = forward - rotation;
        
        double denominator = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower /= denominator;
        rightPower /= denominator;
    }

    public void setDriveVector(Vector2d vector){
        if(driveMode != DriveMode.ByVector) return;
        
        rotationPid.setPID(rotationPIDCoefficients.p, rotationPIDCoefficients.i, rotationPIDCoefficients.d);
        
        double rotation = rotationPid.calculate(localizer.getPose().getHeading(), vector.getAngle());
        double forward = vector.getMagnitude();

        rightPower = forward + rotation;
        leftPower = forward - rotation;

        double denominator = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower /= denominator;
        rightPower /= denominator;
    }
    
    private void updateMotors(){
        left1.setPower(leftPower);
        left2.setPower(leftPower);
        right1.setPower(rightPower);
        right2.setPower(rightPower);

        left1.update();
        left2.update();
        right1.update();
        right2.update();
    }

    @Override
    public void update() {
        localizer.update();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        leftPower = 0;
        rightPower = 0;
        updateMotors();
    }
}
