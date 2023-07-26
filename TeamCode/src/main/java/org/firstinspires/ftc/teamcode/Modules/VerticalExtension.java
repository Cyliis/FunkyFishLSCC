package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Utils.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Utils.CoolEncoder;
import org.firstinspires.ftc.teamcode.Utils.CoolMotor;
import org.firstinspires.ftc.teamcode.Utils.CoolServo;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class VerticalExtension implements IRobotModule {
    private CoolMotor motor1, motor2;
    public static String motor1Name = "lift1", motor2Name = "lift2";
    public static boolean motor1Reversed = false, motor2Reversed = true;
    private CoolEncoder encoder;
    public static String encoderName = "lift1";
    public static boolean encoderReversed = false;

    //units will be in meters
    public static final double ticksPerRev = 28, gearRatio = 5.2, pulleyDiameter = 35.65/1000.0;
    public static final double ticksPerMeter = ticksPerRev*gearRatio/(pulleyDiameter*PI);

    public static double inPosition = 0;
    public static double maxOutPosition = 1.2;
    public static double intermediaryOutPosition = 0.9;
    public static double currentOutPosition = intermediaryOutPosition;

    public static double maxSpeed = ticksPerMeter/((ticksPerRev*gearRatio)/(6000.0/gearRatio));

    public static double upVelocityPercent = 0.9, downVelocityPercent = 0.75;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);
    public static double ff1 = 0.1, ff2 = 0.001;

    private final AsymmetricMotionProfile profile = new AsymmetricMotionProfile(maxSpeed*upVelocityPercent, maxSpeed*downVelocityPercent);

    public enum State{
        In(inPosition), GoingIn(inPosition), Out(currentOutPosition), GoingOut(currentOutPosition);

        double pos;

        State(double pos){
            this.pos = pos;
        }

        static void updatePositions(){
            In.pos = inPosition;
            Out.pos = currentOutPosition;
        }
    }

    private State state = State.In;

    public VerticalExtension(HardwareMap hm){
        motor1 = new CoolMotor(hm, motor1Name, CoolMotor.RunMode.PID, motor1Reversed);
        motor2 = new CoolMotor(hm, motor2Name, CoolMotor.RunMode.PID, motor2Reversed);

        encoder = new CoolEncoder(hm, encoderName, encoderReversed);
    }

    private void setState(State state){
        if(state == this.state) return;
        this.state = state;
        switch (state){
            case GoingIn:
                profile.setMotion(encoder.getCurrentPosition(), State.In.pos);
                break;
            case GoingOut:
                profile.setMotion(encoder.getCurrentPosition(), State.Out.pos);
                break;
        }
    }

    public State getState(){
        return state;
    }

    public static int positionTolerance = 5;

    private void updateState(){
        switch (this.state){
            case In:
            case Out:
                break;
            case GoingIn:
                State.GoingIn.pos = profile.getProfilePosition();
                if(Math.abs(encoder.getCurrentPosition() * ticksPerMeter - State.In.pos) <= positionTolerance)
                    setState(State.In);
                break;
            case GoingOut:
                State.GoingOut.pos = profile.getProfilePosition();
                if(Math.abs(encoder.getCurrentPosition() * ticksPerMeter - State.Out.pos) <= positionTolerance)
                    setState(State.Out);
                break;
        }
    }

    private void updateMotors(){
        double current = encoder.getCurrentPosition()/ticksPerMeter;
        motor1.setPIDF(pidCoefficients, ff1 + ff2*current);
        motor2.setPIDF(pidCoefficients, ff1 + ff2*current);
        motor1.calculatePower(current, state.pos);
        motor2.calculatePower(current, state.pos);
        motor1.update();
        motor2.update();
    }

    @Override
    public void update() {
        State.updatePositions();
        updateState();
        updateMotors();
    }

    @Override
    public void emergencyStop() {

    }
}
