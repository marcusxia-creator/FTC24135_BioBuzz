package org.firstinspires.ftc.teamcode.src.UniversalTools;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class RobotActionConfig {

    public static double flowerRampDownPos          = 0.44;
    public static double flowerRampUpPos            = 1.0;
    public static double gateDownPos                = 0.055;
    public static double gateUpPos                  = 0.11;
    public static double pollenDownPos              = 0.12;
    public static double pollenUpPos                = 0.0;

    ///shooter values
    public static int RPM1 = 2000;
    public static int RPM0 = 1500;

    ///distance values
    public static double closeEdge = 20;
    public static double close = 30;
    public static double far = 40;
    public static double farEdge = 50;

    ///Pose Values
    public static Pose2D redAllianceCell1 = new Pose2D(DistanceUnit.INCH, 13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D redAllianceCell2 = new Pose2D(DistanceUnit.INCH, -13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell1 = new Pose2D(DistanceUnit.INCH, -13, 12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell2 = new Pose2D(DistanceUnit.INCH, 13, 12.75, AngleUnit.DEGREES, 0);

}
