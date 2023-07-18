package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.CoolDigitalSensor;
import org.firstinspires.ftc.teamcode.Utils.CoolEncoder;
import org.firstinspires.ftc.teamcode.Utils.CoolMotor;
import org.firstinspires.ftc.teamcode.Utils.CubicSpline2dMotionProfile;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class DiffyCrane implements IRobotModule {
    private final CoolMotor motor1, motor2;
    public CoolEncoder encoder1, encoder2;

    public static String motor1Name = "crane1", motor2Name = "crane2";
    public static String encoder1Name = "crane1", encoder2Name = "crane2";

    public static boolean motor1Reversed = false, motor2Reversed = true;
    public static boolean encoder1Reversed = false, encoder2Reversed = false;

    HorizontalExtension horizontalExtension;
    VerticalExtension verticalExtension;

    //TODO: calculate these values

    public static double positionTolerance = 5;

    public static double timeToScoringPosition = 1;

    public double getCurrentHorizontalPosition(){
        return encoder1.getCurrentPosition() - encoder2.getCurrentPosition();
    }

    public double getCurrentVerticalPosition(){
        return (encoder1.getCurrentPosition() + encoder2.getCurrentPosition())/2.0;
    }

    private double targetHorizontalPosition, targetVerticalPosition;

    public void setTargetHorizontalPosition(double targetHorizontalPosition){
        this.targetHorizontalPosition = targetHorizontalPosition;
    }

    public void setTargetVerticalPosition(double targetVerticalPosition){
        this.targetVerticalPosition = targetVerticalPosition;
    }

    public double getTargetHorizontalPosition(){
        return targetHorizontalPosition;
    }

    public double getTargetVerticalPosition(){
        return targetVerticalPosition;
    }

    public enum State{
        Down, GoingDown, Score, GoingScore
    }

    public DiffyCrane(HardwareMap hm){
        motor1 = new CoolMotor(hm, motor1Name, CoolMotor.RunMode.PID, motor1Reversed);
        motor2 = new CoolMotor(hm, motor2Name, CoolMotor.RunMode.PID, motor2Reversed);
        encoder1 = new CoolEncoder(hm, encoder1Name, encoder1Reversed);
        encoder2 = new CoolEncoder(hm, encoder2Name, encoder2Reversed);

        horizontalExtension = new HorizontalExtension(hm, this);
        verticalExtension = new VerticalExtension(hm, this);
    }

    public ElapsedTime timeSinceLastStateChange;

    private State currentState = State.Down;

    private CubicSpline2dMotionProfile spline = new CubicSpline2dMotionProfile(
            new Vector2d(0,0), new Vector2d(0, VerticalExtension.scorePosition),
                    new Vector2d(HorizontalExtension.scorePosition, VerticalExtension.scorePosition), timeToScoringPosition);

    private void updateSpline(){
        this.spline = new CubicSpline2dMotionProfile(new Vector2d(0,0), new Vector2d(0, VerticalExtension.scorePosition),
                new Vector2d(HorizontalExtension.scorePosition, VerticalExtension.scorePosition), timeToScoringPosition);
    }

    public void setState(State state){
        if(currentState == state) return;
        switch (state){
            case GoingScore:
                horizontalExtension.setState(HorizontalExtension.State.GoingScore);
                verticalExtension.setState(VerticalExtension.State.GoingScore);

                HorizontalExtension.State.GoingScore.setPos(spline.interpolate(0).getX());
                VerticalExtension.State.GoingScore.setPos(spline.interpolate(0).getY());
                break;
            case GoingDown:
                horizontalExtension.setState(HorizontalExtension.State.GoingIn);
                break;
        }
        this.currentState = state;
        timeSinceLastStateChange.reset();
    }

    public State getState(){
        return currentState;
    }

    @Override
    public void updateState() {
        switch (currentState){
            case Down:
            case Score:
                break;
            case GoingDown:
                if(horizontalExtension.getState() == HorizontalExtension.State.In && verticalExtension.getState() == VerticalExtension.State.In)
                    setState(State.Down);
                if(horizontalExtension.getState() == HorizontalExtension.State.In && (verticalExtension.getState() == VerticalExtension.State.Score || verticalExtension.getState() == VerticalExtension.State.GoingScore)){
                    verticalExtension.setState(VerticalExtension.State.GoingIn);
                }
                break;
            case GoingScore:
                HorizontalExtension.State.GoingScore.setPos(spline.interpolate(timeSinceLastStateChange.seconds()).getX());
                VerticalExtension.State.GoingScore.setPos(spline.interpolate(timeSinceLastStateChange.seconds()).getY());
                if(horizontalExtension.getState() == HorizontalExtension.State.Score && verticalExtension.getState() == VerticalExtension.State.Score)
                    setState(State.Score);
                break;
        }
    }

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);

    public void updateMotors(){
        motor1.setPID(pidCoefficients);

        double motor1Target = getTargetVerticalPosition() + getTargetHorizontalPosition()/2.0;
        double motor2Target = getTargetVerticalPosition() - getTargetHorizontalPosition()/2.0;

        motor1.calculatePower(encoder1.getCurrentPosition(), motor1Target);
        motor2.calculatePower(encoder2.getCurrentPosition(), motor2Target);
    }

    @Override
    public void update() {
        updateSpline();
        updateState();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        motor1.setMode(CoolMotor.RunMode.RUN);
        motor2.setMode(CoolMotor.RunMode.RUN);

        motor1.setPower(0);
        motor2.setPower(0);
    }

}
