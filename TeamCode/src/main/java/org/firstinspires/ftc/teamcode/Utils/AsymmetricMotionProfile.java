package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AsymmetricMotionProfile {
    ElapsedTime timer;

    private double start, end;
    private final double maxPositiveRate, maxNegativeRate;

    public AsymmetricMotionProfile(double maxPositiveRate, double maxNegativeRate){
        this.maxPositiveRate = maxPositiveRate;
        this.maxNegativeRate = maxNegativeRate;
    }

    public void setMotion(double start, double end){
        this.start = start;
        this.end = end;
        timer.reset();
    }

    public double getProfilePosition(){
        if(start <= end) return Math.min(end, start + maxPositiveRate * timer.seconds());
        return Math.max(end, start - maxNegativeRate * timer.seconds());
    }

    public double getTimeToMotionEnd(){
        if(start <= end) return (end-start)/maxPositiveRate;
        return (start - end)/maxNegativeRate;
    }
}
