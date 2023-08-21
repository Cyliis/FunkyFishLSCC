package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AsymmetricMotionProfile {
    private final double acceleration, deceleration;
    private final double maxVelocity;
    private double initialVelocity;
    private double initialPosition, finalPosition;
    private double deltaPose, sign;
    private double maxReachedVelocity;
    private double t0,t1,t2,t3,t;
    private double v0;

    private double position, velocity, signedVelocity;

    private final ElapsedTime timer = new ElapsedTime();

    public AsymmetricMotionProfile(double maxVelocity, double acceleration, double deceleration){
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.deceleration = deceleration;

        setMotion(0,0,0);
    }

    public void setMotion(double initialPosition, double finalPosition, double initialVelocity){
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.initialVelocity = Math.signum(initialVelocity) * Math.min(Math.abs(initialVelocity), maxVelocity);
        sign = Math.signum(finalPosition - initialPosition);
        v0 = sign * initialVelocity;
        t0 = v0/acceleration;
        deltaPose = t0*v0/2.0 + Math.abs(finalPosition - initialPosition);
        maxReachedVelocity = Math.max((calculateDeltaIfMaxReachedVelocityIs(maxVelocity) - deltaPose) <= 0 ? maxVelocity :
                Math.sqrt(deltaPose*2.0*acceleration*deceleration/(acceleration+deceleration)), v0);
        t1 = maxReachedVelocity/acceleration - t0;
        t3 = maxReachedVelocity/deceleration;
        t2 = Math.abs(Math.min(0, calculateDeltaIfMaxReachedVelocityIs(maxVelocity)-deltaPose))/maxVelocity;
        t=t1+t2+t3;
        timer.reset();
    }

    private double calculateDeltaIfMaxReachedVelocityIs(double v){
        return (v*v/2.0)*(acceleration+deceleration)/acceleration*deceleration;
    }

    private int getPhase(){
        if(timer.seconds() <= t1) return 1;
        if(timer.seconds() <= t2) return 2;
        if(timer.seconds() <= t3) return 3;
        return 0;
    }

    private double getVelocity(int phase, double time){
        switch (phase){
            case 1: return v0+time*acceleration;
            case 2: return getVelocity(1, t1);
            case 3: return maxReachedVelocity - deceleration * (timer.seconds() - t1 - t2);
        }
        return 0;
    }

    private double getPosition(double time){
        if(time <= t1) return initialPosition + sign*v0*time/2.0 + sign*getVelocity(1,time)*time/2.0;
        if(time <= t1+t2) return getPosition(t1) + sign*getVelocity(2, time)*(time-t1);
        if(time <= t1+t2+t3) return getPosition(t1+t2) + sign*maxReachedVelocity*t3/2.0 - sign*getVelocity(3, time) * (t1+t2+t3-time)/2.0;
        return getPosition(t1+t2+t3);
    }

    private void updateVelocity(){
        this.velocity = getVelocity(getPhase(), timer.seconds());
    }

    private void updateSignedVelocity(){
        this.signedVelocity = sign * getVelocity(getPhase(), timer.seconds());
    }

    private void updatePosition(){
        this.position = getPosition(timer.seconds());
    }

    public double getVelocity(){
        return velocity;
    }

    public double getSignedVelocity(){
        return signedVelocity;
    }

    public double getPosition(){
        return position;
    }

    public double getTimeToMotionEnd(){
        return t - timer.seconds();
    }

    public void update(){
        updateVelocity();
        updateSignedVelocity();
        updatePosition();
        if(timer.seconds() >= t && position != finalPosition){
            setMotion(position, finalPosition, signedVelocity);
        }
    }
}
