package org.firstinspires.ftc.teamcode.src.UniversalTools;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {

    public static double flowerRampDownPos          = 0.44;
    public static double flowerRampUpPos            = 0.5;
    public static double gateDownPos                = 0.145;
    public static double gateUpPos                  = 0.21;
    public static double pollenDownPos              = 0.57;
    public static double pollenUpPos                = 0.72;
    public static double intakeTargetRPM            = 500.0;
    public static double intakeIdleSpeed            = 0.0;
    public static double intakeTransferRPM          = 600;
    public static double transferTargetRPM          = 400.0;
    public static double IntakeTransferTargetRPM    = 100.0;
    public static double transferToleranceRPM       = 150.0;
    public static double shooterTargetRPM            = 3430.0;
    public static double shooterToleranceRPM         = 100;

    public static double maxShooterRpm              = 5200.0;
    public static double maxFeederRpm               = 1150*0.8;
    public static double REF_VOLTAGE                = 13.1;

        ///Pose Values
    public static Pose2D redAllianceCell1 = new Pose2D(DistanceUnit.INCH, 13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D redAllianceCell2 = new Pose2D(DistanceUnit.INCH, -13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell1 = new Pose2D(DistanceUnit.INCH, -13, 12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell2 = new Pose2D(DistanceUnit.INCH, 13, 12.75, AngleUnit.DEGREES, 0);

}
