package org.firstinspires.ftc.teamcode.src.UniversalTools;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.in;

@Config
public class RobotActionConfig {

    // Intake subsystem
    public static double flowerRampDownPos          = 0.44;
    public static double flowerRampUpPos            = 0.85;
    public static double gateDownPos                = 0.145;
    public static double gateUpPos                  = 0.21;
    public static double pollenDownPos              = 0.57;
    public static double pollenUpPos                = 0.67;
    public static double intakeShootingPower        = 0.7;
    public static double intakeIdleSpeed            = 0.0;
    public static double intakeInitialPower         = 0.7;
    public static double intakeKp                   = 0.001;
    public static double intakeKi                   = 0.0;
    public static double intakeKd                   = 0.0;

    // Shooter subsystem
    public static double shooterVelocityToleranceRPM = 50.0;
    public static double shooterReferenceStableDeltaRPM = 10.0;
    public static int shooterReferenceStableSamples = 8;
    public static double shooterGearRatio            = 30.0 / 40.0;
    public static double shooterTicksPerRev          = 28.0;
    public static double rightShooterKp              = 0.13;
    public static double rightShooterKi              = 0.000033;
    public static double rightShooterKd              = 0.010;
    public static double rightShooterKf              = 0.96;
    public static double rightShooterTargetRPM       = 3430.0;
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

}
