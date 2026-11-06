package org.firstinspires.ftc.teamcode.src.Subsystems;

import static org.apache.commons.math3.util.FastMath.signum;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

/// An example IceWaddler drive train object, built on our robot hardwaremap.<br>
/// If also using a hardwaremap, change class and motor names to match<br>
/// If not using a hardware map, modify the constructor to input and store the four motors as individual parameters
@Config
public class DriveTrain implements IWDriveTrain {
    public static double k_v    =9;
    public static double k_vAng =2;
    public static double k_s    =0;
    public static double k_sAng =0;
    public static double k_a    =1.6;
    public static double k_aAng =0.4;
    public static double maxAllowableLinPower=11;
    public static Scalar wheelPivotRadius       = new Scalar(10, in); //The distance between the pivot point and each of the wheels, or half the length of the diagonal
    RobotHardware robot;
    ///In this implimentation, the constructor simply stores our hardware map
    public DriveTrain(RobotHardware robot){
        this.robot=robot;
    }

    @Override
    /// IW needs motors in float
    public void init() {
        robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void run(DimlessVector linPower, double angPower) {
        if(linPower.mag()>maxAllowableLinPower){
            linPower=linPower.unitVector().multi(maxAllowableLinPower);
        }
        double forward=linPower.getY();
        double strafe=linPower.getX();
        double rot=angPower;

        runVoltage(
                forward+strafe+rot,
                forward-strafe+rot,
                forward-strafe-rot,
                forward+strafe-rot
        );
    }

    @Override
    public void run(Velocity targetVelocity, Acceleration targetAcceleration, NormalizedAngle currentHeading){
        DimlessVector linPower=
                targetVelocity.getLinVel().getDimlessVector().multi(k_v)    //k_v, to counteract motor resistance
                        .add(targetVelocity.getLinVel().mag().getValueSI()==0?new DimlessVector(0,0):targetVelocity.getLinVel().unitVector().multi(k_s))//k_s, to counteract rolling friction
                        .add(targetAcceleration.getLinAcc().getDimlessVector().multi(k_a))//k_a, to accelerate robot
                        .rotateBy(currentHeading.multiply(-1));
        double angPower=
                k_vAng*targetVelocity.getAngVel().getValueSI()+
                k_sAng*(targetVelocity.getAngVel().getValueSI()==0?0:signum(targetVelocity.getAngVel().getValueSI()))+
                k_aAng*targetAcceleration.getAngAcc().getValueSI();
        run(linPower,angPower);
    }

    @Override
    public void runPower(double FL_Power, double BL_Power, double FR_Power, double BR_Power) {
        robot.frontLeftMotor.setPower(FL_Power);
        robot.backLeftMotor.setPower(BL_Power);
        robot.frontRightMotor.setPower(FR_Power);
        robot.backRightMotor.setPower(BR_Power);
    }

    public void runVoltage(double FL_Voltage, double BL_Voltage, double FR_Voltage, double BR_Voltage){
        double voltage=robot.voltageSensor.getVoltage();
        runPower(FL_Voltage/voltage,BL_Voltage/voltage,FR_Voltage/voltage,BR_Voltage/voltage);
    }
}
