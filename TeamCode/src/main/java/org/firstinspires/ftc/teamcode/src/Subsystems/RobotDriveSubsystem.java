package org.firstinspires.ftc.teamcode.src.Subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

public class RobotDriveSubsystem extends SubsystemBase {
    private static final double DRIVE_LERP = 0.35;
    double lastForward;
    double lastStrafe;
    double lastTurn;
    private RobotHardware robot;
    public RobotDriveSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void mecanumDrive (double forward, double strafe, double turn){
        lastForward += DRIVE_LERP * (forward - lastForward);
        lastStrafe += DRIVE_LERP * (strafe - lastStrafe);
        lastTurn += DRIVE_LERP * (turn - lastTurn);

        double FLPower = lastForward + lastStrafe + lastTurn;
        double FRPower = lastForward - lastStrafe - lastTurn;
        double BLPower = lastForward - lastStrafe + lastTurn;
        double BRPower = lastForward + lastStrafe - lastTurn;

        double max = Math.max(Math.abs(FLPower), Math.max(Math.abs(FRPower),
                Math.max(Math.abs(BLPower), Math.abs(BRPower))));
        if (max > 1.0) {
            FLPower /= max;
            FRPower /= max;
            BLPower /= max;
            BRPower /= max;
        }

        robot.frontLeftMotor.setPower(FLPower);
        robot.frontRightMotor.setPower(FRPower);
        robot.backLeftMotor.setPower(BLPower);
        robot.backRightMotor.setPower(BRPower);
    }
}
