package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionParallel.TERMINATIONTYPE.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;

@Autonomous(name="IceWaddler Auto")
public class IWAuto extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;

    Action path;

    //Pathing Positions
    Position startPos=new Position(new Vector(0,0,m),new NormalizedAngle(0,deg));
    Position shootingPos=new Position(new Vector(-0.3,0.5,m),new NormalizedAngle(180,deg));
    Position intakePos=new Position(new Vector(0.4,0.1,m),new NormalizedAngle(135,deg));
    Position parkPos=new Position(new Vector(-0.3,0.1,m),new NormalizedAngle(0,deg));

    IntakeSubsystem intake;
    ShooterSubsystem shooter;

    @Override
    public void init() {
        robot=new RobotHardware();
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(startPos,true);

        intake=new IntakeSubsystem(robot);
        shooter=new ShooterSubsystem(robot);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        path=new ActionSeries(
                waddler.new InitPath(new PathingPoint(startPos)),
                new ActionParallel(FIRST,
                        waddler.new MotionAction(new Line(new PathingPoint(shootingPos),new maxSpeedMP(),new linearHP(),new String[]{})),
                        shooter.new SpinFlywheel()
                ),
                new ActionParallel(FIRST,
                        new Delay(3),
                        new ActionParallel(NONE,
                                shooter.new ShootAction(),
                                waddler.new MotionAction(new holdPos(new String[]{})))
                ),
                new ActionParallel(FIRST,
                        waddler.new MotionAction(new Line(new PathingPoint(intakePos),
                                new LUT_MP(new double[]{0.7},new Scalar[]{new Scalar(0.3,metersPerSecond)},new MotionProfile[]{new maxSpeedMP(),new maxSpeedMP(new Scalar(0.3,metersPerSecond))}),
                                new linearHP(),new String[]{})),
                        intake.new IntakeAction()
                ),
                new ActionParallel(FIRST,
                        waddler.new MotionAction(new Line(new PathingPoint(shootingPos),new maxSpeedMP(),new linearHP(),new String[]{})),
                        shooter.new SpinFlywheel()
                ),
                new ActionParallel(FIRST,
                        new Delay(3),
                        new ActionParallel(NONE,
                                shooter.new ShootAction(),
                                waddler.new MotionAction(new holdPos(new String[]{})))
                ),
                waddler.new MotionAction(new Line(new PathingPoint(parkPos),
                        new LUT_MP(new double[]{0.7},new Scalar[]{new Scalar(0.3,metersPerSecond)},new MotionProfile[]{new maxSpeedMP(),new maxSpeedMP(new Scalar(0.3,metersPerSecond))}),
                        new linearHP(),new String[]{}))
        );
        
        rootAction=new ActionParallel(NONE,
                path,
                new telemetryDriver()
        );
    }

    class telemetryDriver implements Action{
        public telemetryDriver(){}

        @Override
        public void loop() {
            waddler.drawField();
            telemetry.addData("Ticktime",waddler.getTickTime().getValueSI());

            telemetry.addData("1a.Current x pos", waddler.getCurrentSituation().getPosition().getX().getValueSI());
            telemetry.addData("1a.Current y pos", waddler.getCurrentSituation().getPosition().getY().getValueSI());
            telemetry.addData("1a.Current heading", waddler.getCurrentSituation().getPosition().getHeading().getValueSI());

            if(waddler.getTargetSituation().getPosition()!=null) {
                telemetry.addData("1b.Target x pos", waddler.getTargetSituation().getPosition().getX().getValueSI());
                telemetry.addData("1b.Target y pos", waddler.getTargetSituation().getPosition().getY().getValueSI());
                telemetry.addData("1b.Target heading", waddler.getTargetSituation().getPosition().getHeading().getValueSI());
            }

            telemetry.addData("2a.Current x vel", waddler.getCurrentSituation().getVelocity().getX().getValueSI());
            telemetry.addData("2a.Current y vel", waddler.getCurrentSituation().getVelocity().getY().getValueSI());
            telemetry.addData("2a.Current ang vel", waddler.getCurrentSituation().getVelocity().getAngVel().getValueSI());

            if(waddler.getTargetSituation().getVelocity()!=null) {
                telemetry.addData("2b.Target x vel", waddler.getTargetSituation().getVelocity().getX().getValueSI());
                telemetry.addData("2b.Target y vel", waddler.getTargetSituation().getVelocity().getY().getValueSI());
                telemetry.addData("2b.Target ang vel", waddler.getTargetSituation().getVelocity().getAngVel().getValueSI());
            }

            telemetry.addData("3a.Current x acc", waddler.getCurrentSituation().getAcceleration().getX().getValueSI());
            telemetry.addData("3a.Current y acc", waddler.getCurrentSituation().getAcceleration().getY().getValueSI());
            telemetry.addData("3a.Current ang acc", waddler.getCurrentSituation().getAcceleration().getAngAcc().getValueSI());

            if(waddler.getLastTargetSituation().getAcceleration()!=null) {
                telemetry.addData("3b.Target x acc", waddler.getTargetSituation().getAcceleration().getX().getValueSI());
                telemetry.addData("3b.Target y acc", waddler.getTargetSituation().getAcceleration().getY().getValueSI());
                telemetry.addData("3b.Target ang acc", waddler.getTargetSituation().getAcceleration().getAngAcc().getValueSI());
            }

            if(waddler.getCurrentAction()!=null) {
                telemetry.addData("4.Completion",waddler.getCurrentAction().getCompletion());
            }

            //telemetry.addData("4.Target Pathingpoint exists",waddler.targetPathingPoint!=null);

            telemetry.update();
        }
    }
}
