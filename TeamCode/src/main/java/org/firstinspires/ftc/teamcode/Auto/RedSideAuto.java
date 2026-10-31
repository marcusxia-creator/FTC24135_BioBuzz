package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionParallel.TERMINATIONTYPE.*;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.lib.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltPathingElements.*;

@Config
@Autonomous(name="Red Side Auto")
public class RedSideAuto extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;

    Action path;

    //Pathing Positions
    public static Position startPoint=new Position(new Vector(5.2,-1,feet),new NormalizedAngle(-90,deg));
    public static Position leftHive=new Position(new Vector(4,-1,feet),new NormalizedAngle(-90,deg));
    public static Position rightHive=new Position(new Vector(-4,-1,feet),new NormalizedAngle(90,deg));
    public static Position flower1=new Position(new Vector(2,-4.7,feet),new NormalizedAngle(180,deg));
    public static Position flower2=new Position(new Vector(-4.7,-2,feet),new NormalizedAngle(-90,deg));
    public static Position parkPoint=new Position(new Vector(-4.3,-4.6,feet),new NormalizedAngle(-90,deg));

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init();

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(startPoint,false);

        path=new ActionSeries(
                waddler.new InitPath(new PathingPoint(startPoint)),
                new ActionParallel(ALL,//spool flywheel
                        waddler.new MotionAction(new Line(new PathingPoint(leftHive), new maxSpeedMP(), new linearHP(), new String[]{}))),
                new ActionParallel(FIRST,//shoot
                        waddler.new MotionAction(new holdPos(new String[]{}))),

                new ActionParallel(ALL,//put ramp down
                        waddler.new MotionAction(new Line(new PathingPoint(flower1), new maxSpeedMP(), new linearHP(), new String[]{}))),
                new ActionParallel(FIRST,new Delay(5),//intake
                        waddler.new MotionAction(new holdPos(new String[]{}))),
                new ActionParallel(ALL,
                        waddler.new MotionAction(new Line(new PathingPoint(new Position(new Vector(rightHive.getX(),new Scalar(-3,feet)),rightHive.getHeading()),new Scalar(0.3,metersPerSecond)), new maxSpeedMP(), new linearHP(), new String[]{})),
                        new ActionSeries(
                                new Delay(0.4)//,retract ramp
                                )),
                new ActionParallel(ALL,//spool flywheel
                        waddler.new MotionAction(new Line(new PathingPoint(rightHive), new maxSpeedMP(), new linearHP(), new String[]{}))),
                new ActionParallel(FIRST,//shoot
                        waddler.new MotionAction(new holdPos(new String[]{}))),

                new ActionParallel(ALL,//put ramp down
                        waddler.new MotionAction(new Line(new PathingPoint(flower2), new maxSpeedMP(), new linearHP(), new String[]{}))),

                new ActionParallel(ALL,//spool flywheel,
                        new ActionSeries(
                                new Delay(0.4)//,retract ramp
                        )),
                        waddler.new MotionAction(new Line(new PathingPoint(rightHive), new maxSpeedMP(), new linearHP(), new String[]{})),
                new ActionParallel(FIRST,//shoot
                        waddler.new MotionAction(new holdPos(new String[]{}))),

                waddler.new MotionAction(new Line(new PathingPoint(parkPoint), new maxSpeedMP(), new linearHP(), new String[]{}))
        );

        rootAction=new ActionParallel(NONE,
                path//,telemetry or smth
        );
    }
}
