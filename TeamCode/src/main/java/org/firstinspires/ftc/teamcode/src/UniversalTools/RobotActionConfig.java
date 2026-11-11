package org.firstinspires.ftc.teamcode.src.UniversalTools;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class RobotActionConfig {

    public static double flowerRampDownPos          = 0.44;
    public static double flowerRampUpPos            = 1.0;
    public static double gateDownPos                = 0.145;
    public static double gateUpPos                  = 0.21;
    public static double pollenDownPos              = 0.57;
    public static double pollenUpPos                = 0.67;
    public static double intakeShootingPower        = 0.7;
    public static double intakeIdleSpeed            = 0.0;

    public static double maxVelocityRPM             = 5200.0;
    public static double REF_VOLTAGE                =13.1;


    ///Pose Values
    public static Pose2D redAllianceCell1 = new Pose2D(DistanceUnit.INCH, 13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D redAllianceCell2 = new Pose2D(DistanceUnit.INCH, -13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell1 = new Pose2D(DistanceUnit.INCH, -13, 12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell2 = new Pose2D(DistanceUnit.INCH, 13, 12.75, AngleUnit.DEGREES, 0);
}
