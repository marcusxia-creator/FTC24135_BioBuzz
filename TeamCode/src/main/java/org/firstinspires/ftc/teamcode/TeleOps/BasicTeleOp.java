package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionParallel;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionSeries;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionSwitch;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.Standby;
import org.firstinspires.ftc.teamcode.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.linearHP;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.maxSpeedMP;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements.Line;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements.holdPos;
import org.firstinspires.ftc.teamcode.Subsystems.InputSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp
public class BasicTeleOp extends ScheduledOpMode {
    InputSubsystem driverInput;

    RobotHardware robot;

    IceWaddler waddler;

    IntakeSubsystem intake;
    ShooterSubsystem shooter;

    @Override
    public void init() {
        driverInput=new InputSubsystem(gamepad1,gamepad2);

        robot=new RobotHardware();
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(Position.ORIGIN,false);

        intake=new IntakeSubsystem(robot);
        shooter=new ShooterSubsystem(robot);

        rootAction=new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                driverInput.new Update(),
                new ActionSwitch(driverInput.actionSelect,
                        new Standby(),
                        intake.new IntakeAction(),//a to intake
                        shooter.new ShootAction()// y to shoot
                ),
                new ActionSwitch(driverInput.driveSelect,
                        waddler.new VelDrive(driverInput.drive),//manual drive
                        new ActionSeries(
                                waddler.new InitPath(),
                                waddler.new MotionAction(new Line(new PathingPoint(Position.ORIGIN),new maxSpeedMP(), new linearHP(),new String[0])),
                                new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,waddler.new MotionAction(new holdPos(new String[0])))//Use ActionParllel to keep waddler from closing
                        )
                )
        );
    }
}
