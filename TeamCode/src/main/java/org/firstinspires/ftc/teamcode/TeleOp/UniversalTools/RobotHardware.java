package org.firstinspires.ftc.teamcode.TeleOp.UniversalTools;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware{
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx leftShooterMotor;
    public DcMotorEx rightShooterMotor;
    //public DcMotorEx flowerMechMotor;
    public Servo transferServo;
    public Servo rampServo;
    public Servo limelightAdjServo;
    public Servo transferLimitingServo;
    public DistanceSensor distanceSensor;
    public HardwareMap hardwareMap;
    public RobotHardware(HardwareMap hardwareMap) {
       this.hardwareMap=hardwareMap;
    }
    public void init(){
        ///Drive
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ///Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ///Shooter
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "Left_Shooter_Motor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "Right_Shooter_Motor");

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///Servos
        transferServo = hardwareMap.get(Servo.class, "Transfer_Servo");
        ///Sensors
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance_Sensor");
    }
}
