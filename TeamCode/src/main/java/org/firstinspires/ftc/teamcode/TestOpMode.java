package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Wrappers.CoolEncoder;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "bru")
public class TestOpMode extends OpMode {

    CoolMotor motor1, motor2;
    CoolEncoder encoder;
    ElapsedTime timer = new ElapsedTime();

    List<LynxModule> hubs= new ArrayList<>();

    @Override
    public void init() {
        encoder = new CoolEncoder(hardwareMap, "mfl");
        motor1 = new CoolMotor(hardwareMap, "mfr");
        motor2 = new CoolMotor(hardwareMap, "mbl");
        timer.startTime();
        timer.reset();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    @Override
    public void loop() {
        int nr = encoder.getCurrentPosition();
        motor1.setPower(0.01);
        motor2.setPower(0.01);
        motor1.update();
        motor2.update();
        telemetry.addData("hz",1.0/timer.seconds());
        timer.reset();
        for (LynxModule hub: hubs) hub.clearBulkCache();
    }
}
