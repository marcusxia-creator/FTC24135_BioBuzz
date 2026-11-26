package org.firstinspires.ftc.teamcode.src.UniversalTools;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.in;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class RobotActionConfig {

    // Intake subsystem
    public static double flowerRampDownPos          = 0.16; //0.04
    public static double flowerRampUpPos            = 0.76; //0.67
    public static double gateDownPos                = 0.145;
    public static double gateUpPos                  = 0.21;
    public static double limiterDownPos             = 0.56; //0.58
    public static double limiterUpPos               = 0.72;
    public static double intakeIdleSpeed            = 0.0;
    public static double intakePower                = 0.7;
    public static double intakeKp                   = 0.001;
    public static double intakeKi                   = 0.0;
    public static double intakeKd                   = 0.0;
    public static double intakeGearRatio            = 1.0;
    public static double intakeTicksPerRev          = 28.0;
    // Intake RPM threshold for full/jammed detection.
    public static double intakeFullRPMThreshold     = 300.0;
    public static double intakeTransferRPM          = 600.0;
    public static double intakeTargetRPM            = 600.0;

    // Transfer subsystem
    public static double transferGearRatio          = 1.0;
    public static double transferTicksPerRev        = 28.0;
    public static double transferMaxRPM             = 1150 * 0.85; ///This is placeholder, value need to be determined
    public static double transferIntakePower        = 0.5;
    public static double transferKp                 = 0.13;
    public static double transferKi                 = 0.000033;
    public static double transferKd                 = 0.010;
    public static double transferKf                 = 0.96;
    public static double transferTargetRPM          = 500.0;
    // Transfer RPM threshold for full/jammed detection.

    // Shooter subsystem
    public static double shooterSpoolUpTimeSeconds = 2.0;
    public static double shooterLimiterRetractTimeSeconds = 1.75;
    public static double shooterReadyMinimumTargetFraction = 0.975;
    public static double shooterReadyStabilityWindowSeconds = 0.30;
    public static double shooterGearRatio            = 1; //30 to 40
    public static double shooterTicksPerRev          = 28.0;
    public static double rightShooterKp              = 0.13;
    public static double rightShooterKi              = 0.000033;
    public static double rightShooterKd              = 0.010;
    public static double rightShooterKf              = 0.97;
    public static double shooterTargetVelocityRPM    = 3480.0;
    public static double maxVelocityRPM              = 5200.0;
    public static double REF_VOLTAGE                 = 13.1;

    // Robot drive subsystem
    public static double driveLerp                   = 0.35;

    // IceWaddler drive train subsystem
    public static double k_v                         = 9.0;
    public static double k_vAng                      = 2.0;
    public static double k_s                         = 0.0;
    public static double k_sAng                      = 0.0;
    public static double k_a                         = 1.6;
    public static double k_aAng                      = 0.4;
    public static double maxAllowableLinPower        = 11.0;
    public static Scalar wheelPivotRadius            = new Scalar(10, in);


    ///Pose Values
    public static Pose2D redAllianceCell1 = new Pose2D(DistanceUnit.INCH, 13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D redAllianceCell2 = new Pose2D(DistanceUnit.INCH, -13, -12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell1 = new Pose2D(DistanceUnit.INCH, -13, 12.75, AngleUnit.DEGREES, 0);
    public static Pose2D blueAllianceCell2 = new Pose2D(DistanceUnit.INCH, 13, 12.75, AngleUnit.DEGREES, 0);
}
