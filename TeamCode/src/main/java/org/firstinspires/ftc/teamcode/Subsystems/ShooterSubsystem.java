package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionSeries;

public class ShooterSubsystem {
    DcMotorEx shooterMotor;
    CRServo transferServo;

    public Action shoot;

    public ShooterSubsystem(RobotHardware robot){
        shooterMotor=robot.shooterMotor;
        transferServo= robot.transferServo;
    }

    public class ShootAction implements Action{
        @Override
        public void init() {
            shooterMotor.setVelocity(1000);
        }

        @Override
        public void loop() {
            if(shooterMotor.isBusy()){
                transferServo.setPower(0);
            }
            else{
                transferServo.setPower(-0.5);
            }
        }

        public void shutdown(){
            shooterMotor.setPower(0);
            transferServo.setPower(0);
        }
    }
}
