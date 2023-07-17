package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utils.CoolDigitalSensor;
import org.firstinspires.ftc.teamcode.Utils.CoolEncoder;
import org.firstinspires.ftc.teamcode.Utils.CoolMotor;

@Config
public class DiffyCrane {
    private CoolMotor motor1, motor2;
    public CoolEncoder encoder1, encoder2;
    private CoolDigitalSensor verticalLimitSwitch, horizontalLimitSwitch;

    public static String motor1Name = "crane1", motor2Name = "crane2";
    public static String encoder1Name = "crane1", encoder2Name = "crane2";
    public static String verticalLimitSwitchName = "verticalLimitSwitch", horizontalLimitSwitchName = "horizontalLimitSwitch";

    //TODO: calculate these values

    public static double maxVerticalExtension, maxHorizontalExtension;
    public static double maxVerticalExtensionTime, maxHorizontalExtensionTime;

    public static double scoreVerticalPosition, scoreHorizontalPosition;

    public static double positionTolerance = 5;

    public enum State{
        Down, GoingDown, Score, GoingScore
    }

    private enum HorizontalState{
        In, GoingIn, Out, GoingOut
    }

    private enum VerticalState{
        Down, GoingDown, Up, GoingUp
    }

}
