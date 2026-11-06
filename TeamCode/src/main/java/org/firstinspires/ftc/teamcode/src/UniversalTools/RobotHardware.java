package org.firstinspires.ftc.teamcode.src.UniversalTools;

import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.*;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Hardware.PrebuiltHardware.goBildaOdoComputer;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.src.Subsystems.DriveTrain;

public class RobotHardware {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    public GoBildaPinpointDriver odo;

    public VoltageSensor voltageSensor;

    public DcMotorEx intakeMotor;
    public DcMotorEx leftShooterMotor;
    public DcMotorEx rightShooterMotor;

    public Servo gateServo;
    public Servo leftFlowerRampServo;
    public Servo rightFlowerRampServo;
    public Servo limelightAdjServo;
    public Servo pollenLimitingServo;
    public HardwareMap hardwareMap;

    public IWDriveTrain driveTrain;
    public IWLocalizer localizer;


    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        ///Drive
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");

        //Ion think we really need this
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RunMode mode=RunMode.RUN_WITHOUT_ENCODER;
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);

        ZeroPowerBehavior zeroPowerBehavior=ZeroPowerBehavior.BRAKE;//for better power teleop handling. I do want to move to a IW veldrive later on tho
        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        voltageSensor=hardwareMap.get(VoltageSensor.class,"Control Hub");

        odo=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        driveTrain=new DriveTrain(this);
        localizer=new goBildaOdoComputer(
                odo,
                new Scalar(-25,mm),
                new Scalar(-150,mm),//value larp
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                REVERSED,
                REVERSED
        );

        ///Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        ///Shooter
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "Left_Shooter_Motor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "Right_Shooter_Motor");

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ///Servos
        gateServo = hardwareMap.get(Servo.class, "Gate_Servo");
        pollenLimitingServo = hardwareMap.get(Servo.class, "Limit_Servo");
        leftFlowerRampServo = hardwareMap.get(Servo.class, "Left_Flower_Servo");
        rightFlowerRampServo = hardwareMap.get(Servo.class, "Right_Flower_Servo");
        limelightAdjServo = hardwareMap.get(Servo.class, "Limelight_Servo");

        rightFlowerRampServo.setDirection(Servo.Direction.REVERSE);
        gateServo.setDirection(Servo.Direction.REVERSE);
        pollenLimitingServo.setDirection(Servo.Direction.REVERSE);
    }
}
