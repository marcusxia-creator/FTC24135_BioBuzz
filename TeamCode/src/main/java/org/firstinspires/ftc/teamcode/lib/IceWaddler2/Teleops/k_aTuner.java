package org.firstinspires.ftc.teamcode.lib.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionParallel;
import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionSwitch;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.GamepadInput;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.PrebuiltInputs.AnalogInputs.LerpedVel;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.PrebuiltInputs.AnalogInputs.SmoothedVel;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Vector;
import org.firstinspires.ftc.teamcode.lib.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(name="k_a/Accel Tuner", group="IceWaddler")
@Config
public class k_aTuner extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;
    FtcDashboard dashboard;

    public static Scalar linVelFactor=new Scalar(2.5,metersPerSecond);
    public static Scalar angVelFactor=new Scalar(5,radiansPerSecond);

    GamepadInput<Velocity> driveInput;

    @Override
    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(new Position(new Vector(0,0,m),new NormalizedAngle(0,deg)),false);

        dashboard=FtcDashboard.getInstance();

        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        driveInput=new SmoothedVel(new LerpedVel(gamepad1,gamepad2,(gamepad) -> (double)gamepad.right_stick_y,(gamepad) -> (double)gamepad.right_stick_x,(gamepad) -> (double)gamepad.left_stick_x,linVelFactor,angVelFactor,new Scalar(5,metersPerSecondSquared),new Scalar(5,radiansPerSecondSquared)),2);

        rootAction=new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                new ActionSwitch(()->gamepad1.b?1:0, waddler.new VelDrive(driveInput),
                        waddler.new Idle()),
                new telemetryDriver()
        );
    }

    class telemetryDriver implements Action {
        public telemetryDriver(){}

        @Override
        public void loop() {
            telemetry.addData("Ticktime",waddler.getTickTime().getValueSI());

            telemetry.addData("1.Current x vel",waddler.getCurrentSituation().getVelocity().getX().getValueSI());
            telemetry.addData("1.Current y vel",waddler.getCurrentSituation().getVelocity().getY().getValueSI());
            telemetry.addData("1.Current ang vel",waddler.getCurrentSituation().getVelocity().getAngVel().getValueSI());

            telemetry.addData("2.Target x vel", waddler.getTargetSituation().getVelocity().getX().getValueSI());
            telemetry.addData("2.Target y vel", waddler.getTargetSituation().getVelocity().getY().getValueSI());
            telemetry.addData("2.Target ang vel", waddler.getTargetSituation().getVelocity().getAngVel().getValueSI());

            telemetry.addData("3.Current x acc", waddler.getCurrentSituation().getAcceleration().getX().getValueSI());
            telemetry.addData("3.Current y acc", waddler.getCurrentSituation().getAcceleration().getY().getValueSI());
            telemetry.addData("3.Current ang acc", waddler.getCurrentSituation().getAcceleration().getAngAcc().getValueSI());

            telemetry.addData("4.Target x acc", waddler.getTargetSituation().getAcceleration().getX().getValueSI());
            telemetry.addData("4.Target y acc", waddler.getTargetSituation().getAcceleration().getY().getValueSI());
            telemetry.addData("4.Target ang acc", waddler.getTargetSituation().getAcceleration().getAngAcc().getValueSI());

            telemetry.addData("5.Last Target x vel", waddler.getLastTargetSituation().getVelocity().getX().getValueSI());
            telemetry.addData("5.Last Target y vel", waddler.getLastTargetSituation().getVelocity().getY().getValueSI());
            telemetry.addData("5.Last Target ang vel", waddler.getLastTargetSituation().getVelocity().getAngVel().getValueSI());
        }
    }
}
