package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.Localizer;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.CubicBezierTrajectorySegment;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.Trajectory;
import org.firstinspires.ftc.teamcode.TrajectoryStuff.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Utils.Pose;

import java.util.List;

@Config
@Autonomous(name = "Sample")
public class SampleOpMode extends LinearOpMode {
    List<LynxModule> hubs;

    MecanumDrive drive;
    Follower follower;
    Localizer localizer;

    FtcDashboard dash;

    CubicBezierTrajectorySegment forwardSegment = new CubicBezierTrajectorySegment(
            new Pose(0,0,0),
            new Pose(0,-20,0),
            new Pose(36,30,0),
            new Pose(36,10,0)
    );

    CubicBezierTrajectorySegment rotationSegment = new CubicBezierTrajectorySegment(
            new Pose(0,0,0),
            new Pose(0,0,0),
            new Pose(0,0, PI),
            new Pose(0,0, PI)
    );

    Trajectory forward = new TrajectoryBuilder(forwardSegment).build(),
            rotate = new TrajectoryBuilder(rotationSegment).build();

    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        localizer = new Localizer(hardwareMap, new Pose());
        drive = new MecanumDrive(hardwareMap, localizer);
        follower = new Follower(drive, localizer);
        follower.setTrajectory(forward, 2);

        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        localizer.imu.startIMUThread(this);

        int loops = 0;
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while(opModeIsActive() && !isStopRequested()){
            for(LynxModule hub:hubs)
                hub.clearBulkCache();
            localizer.update();
            follower.update();
            drive.update();
            loops++;
            telemetry.addData("loops", loops);
            telemetry.addData("followed trajectory length",follower.getTrajectory().getLength());
            telemetry.addData("followed point", follower.currentFollowedPoint);
            telemetry.addData("loops/sec", 1.0/loopTimer.seconds());
            telemetry.addData("drive mode", drive.getRunMode());
            telemetry.update();
            loopTimer.reset();
        }
    }
}
