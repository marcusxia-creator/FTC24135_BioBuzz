package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.lib.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.IWConfig;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Vector;
import org.firstinspires.ftc.teamcode.src.Subsystems.*;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@Config
@TeleOp(name = "Basic TeleOp", group = "TeleOp")
public class BasicTeleOp extends ScheduledOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private GamepadInputSubsystem controls;
    private RobotHardware robot;
    private IceWaddler waddler;

    public static Vector leftHive=new Vector(1.4,-1.1,feet);
    public static Vector rightHive=new Vector(-1.4,-1.1,feet);

    public static double targetShooterRPM = 1500;
    public static double actualShooterRPM;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();

        intake = new IntakeSubsystem(robot);
        intake.setInitialServoPositions();
        shooter = new ShooterSubsystem(robot);
        controls = new GamepadInputSubsystem(gamepad1,gamepad2);
        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(Position.ORIGIN,false);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rootAction=new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                controls.new Update(),
                waddler.new VelDrive(()->new Velocity(controls.driveInput.get().getLinVel(),
                        controls.aimLeftHive.get()||controls.aimRightHive.get()?IWConfig.headingController.getCorrection(waddler.getCurrentSituation().getPosition().getHeading().sub(waddler.getCurrentSituation().getPosition().getLinPos().angleTo(waddler.getCurrentSituation().getPosition().getX().greaterThan(new Scalar(0,feet))?leftHive:rightHive)))
                        :controls.driveInput.get().getAngVel())),

                new ActionSwitch(controls.actionSelect,
                        new Standby(),
                        new ActionSwitch(controls.intakeSelect,
                                intake.new pollenIntakeAction(),
                                intake.new nectarIntakeAction(),
                                intake.new flowerIntakeAction()),
                        new ActionSeries(
                                new ActionParallel(ActionParallel.TERMINATIONTYPE.ALL, new ConditionalStandby(controls.shootButton),
                                        shooter.new spoolFlywheelAction()),
                                shooter.new ShootAction(),
                                new Standby()
                        )
                ),
                this::writeTelemetry
        );
    }

    void writeTelemetry(){
        telemetry.addData("Intake Target Power", "%.1f", intake.getIntakeMotorPower());
        telemetry.addData("Intake Active Power", "%.2f", robot.intakeMotor.getPower());
        telemetry.addLine("--- SHOOTER TELEMETRY ---");
        telemetry.addData("Target Speed", "%.0f RPM (%.0f TPS)", shooter.getTargetRPM(), shooter.getTargetTicksPerSecond());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("Actual Speed", "%.0f RPM (%.0f TPS)", shooter.getShooterRPM(), shooter.getShooterTicksPerSecond());
        telemetry.addData("Current RPM", shooter.getShooterRPM());
        telemetry.addData("Shooter power", robot.rightShooterMotor.getPower());
        telemetry.update();
    }
}
