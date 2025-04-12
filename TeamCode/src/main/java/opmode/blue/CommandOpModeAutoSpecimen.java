/** DO NOT USE THIS FILE */

package opmode.blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.function.BooleanSupplier;

import common.commands.follower.followPath;
import common.commands.follower.setMaxPower;
import common.robot.robotHardware;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Auto 5+0 Blue", group = "Blue")
//@Disabled
public class CommandOpModeAutoSpecimen extends CommandOpMode {

    robotHardware robot;
    private Telemetry telemetryA;

    public static double DISTANCE = 5;

    private boolean forward = true;

    private Follower follower;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier toSampleX = () -> follower.getPose().getX()>30 && follower.getPose().getY()>10;
    private BooleanSupplier toHumanX = () -> follower.getPose().getX()>50 && follower.getPose().getY()<-25;

    private Path forwards;
    private Path backwards;
    private Path toDropPreload;
    private Path toDropZoneTwo;
    private Path toDropZoneThree;
    private Path toDropZoneFour;
    private Path toDropZoneFive;
    private Path toSampleOne;
    private Path toSampleTwo;
    private Path toSampleStrafeOne;
    private Path toSampleThree;
    private Path toSampleStrafeTwo;
    private Path toHumanOne;
    private Path toHumanTwo;
    private Path toHumanThree;
    private Path toHumanPick;
    private Path toHumanPickFromDropTwo;
    private Path toHumanPickFromDropThree;
    private Path toHumanPickFromDropFour;
    private Path toHumanPickFromDrop;
//    private boolean goToSampleOne = follower.getPose().getX() > 30 && follower.getPose().getY() > 10;
//    private boolean goToHumanOne = follower.getPose().getX() > 50 && follower.getPose().getY() < -25;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */

    public void initialize() {

        robot = robotHardware.getInstance();
        robot.init(hardwareMap);

//        follower = new Follower(hardwareMap, Constants.fConstants, Constants.lConstants);
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);


        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        toDropPreload= new Path(new BezierCurve(
                new Point(0,0,Point.CARTESIAN),
//                new Point(18,10,Point.CARTESIAN),
                new Point(28,18,Point.CARTESIAN),
                new Point(31.3,19,Point.CARTESIAN)
        )
        );
        toDropPreload.setConstantHeadingInterpolation(Math.toRadians(0));

        toSampleOne= new Path(new BezierCurve(
                new Point(31.3,19,Point.CARTESIAN),
                new Point(28,13,Point.CARTESIAN),
                new Point(20,6,Point.CARTESIAN),
                new Point(20,-13,Point.CARTESIAN),
                new Point(20,-20,Point.CARTESIAN),
                new Point(27,-20,Point.CARTESIAN),
                new Point(34,-20,Point.CARTESIAN),
                new Point(48,-14,Point.CARTESIAN),
                new Point(53,-29,Point.CARTESIAN)

        )
        );
//        toSampleOne.setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(180));
        toSampleOne.setConstantHeadingInterpolation(Math.toRadians(0));
        toHumanOne= new Path(new BezierLine(
                new Point(53,-29,Point.CARTESIAN),
                new Point(10,-29,Point.CARTESIAN)
        )
        );
        toHumanOne.setConstantHeadingInterpolation(Math.toRadians(0));

        toSampleTwo= new Path(new BezierCurve(
                new Point(10,-29,Point.CARTESIAN),
                new Point(53,-29,Point.CARTESIAN),
                new Point(53,-29,Point.CARTESIAN),
                new Point(53,-29,Point.CARTESIAN),
                new Point(53,-38,Point.CARTESIAN)

        )
        );
        toSampleTwo.setConstantHeadingInterpolation(Math.toRadians(0));

        toSampleStrafeOne= new Path(new BezierLine(
                new Point(27,18,Point.CARTESIAN),
                new Point(31.4,18,Point.CARTESIAN)
        )
        );
        toSampleStrafeOne.setConstantHeadingInterpolation(Math.toRadians(0));

        toHumanTwo= new Path(new BezierLine(
                new Point(53,-38,Point.CARTESIAN),
                new Point(10,-38,Point.CARTESIAN)
        )
        );
        toHumanTwo.setConstantHeadingInterpolation(Math.toRadians(0));

//        toSampleThree= new Path(new BezierCurve(
//                new Point(10,-38,Point.CARTESIAN),
//                new Point(56,-38,Point.CARTESIAN),
//                new Point(56,-38,Point.CARTESIAN), //siddy boutta be like wo
//                new Point(56,-38,Point.CARTESIAN),
//                new Point(56,-41,Point.CARTESIAN)
//        )
//        );
//        toSampleThree.setConstantHeadingInterpolation(Math.toRadians(0));

        toSampleStrafeTwo= new Path(new BezierLine(
                new Point(50,-40,Point.CARTESIAN),
                new Point(50,-48.5,Point.CARTESIAN)
        )
        );
        toSampleStrafeTwo.setConstantHeadingInterpolation(Math.toRadians(180));

//        toHumanThree= new Path(new BezierLine(
//                new Point(56,-41,Point.CARTESIAN),
//                new Point(10,-41,Point.CARTESIAN)
//        )
//        );
//        toHumanThree.setConstantHeadingInterpolation(Math.toRadians(0));

        toHumanPick= new Path(new BezierCurve(
                new Point(10,-38,Point.CARTESIAN),
                new Point(17,-25,Point.CARTESIAN),
                new Point(4.5,-25,Point.CARTESIAN)
        )
        );
        toHumanPick.setConstantHeadingInterpolation(Math.toRadians(2));

        toDropZoneTwo= new Path(new BezierCurve(
//                new Point(4,-25,Point.CARTESIAN),

//                new Point(11,-25,Point.CARTESIAN),
//                new Point(20,-25,Point.CARTESIAN),
//                new Point(22,-25,Point.CARTESIAN),
//                new Point(22,-20,Point.CARTESIAN),
////                new Point(22,-16,Point.CARTESIAN),
////                new Point(22,-9,Point.CARTESIAN),
//                new Point(22,12,Point.CARTESIAN),
//                new Point(22,16,Point.CARTESIAN),
////                new Point(25,15,Point.CARTESIAN),
//                new Point(31.15,16,Point.CARTESIAN)
                new Point(5,-25, Point.CARTESIAN),
                new Point(28,10,Point.CARTESIAN),
                new Point(30.5,16.75,Point.CARTESIAN)
        )
        );
        toDropZoneTwo.setConstantHeadingInterpolation(0);

        toHumanPickFromDropTwo= new Path(new BezierCurve(
//                new Point(30,11,Point.CARTESIAN),
//                new Point(25,15.5,Point.CARTESIAN),
//                new Point(17,10.6,Point.CARTESIAN),
//                new Point(11.5,5.78,Point.CARTESIAN),
//                new Point(9.3,3.7,Point.CARTESIAN),
//                new Point(6.9,0.6969,Point.CARTESIAN),
//                new Point(6.9,-25,Point.CARTESIAN),
//                new Point(7,-25,Point.CARTESIAN)
                new Point(31,16.75,Point.CARTESIAN),
                new Point(20,13,Point.CARTESIAN),
//                new Point(10,0,Point.CARTESIAN),
                new Point(10,-25,Point.CARTESIAN),
                new Point(5,-25,Point.CARTESIAN)
        )
        );
        toHumanPickFromDropTwo.setConstantHeadingInterpolation(Math.toRadians(0));

        toDropZoneThree= new Path(new BezierCurve(
//                new Point(4,-25,Point.CARTESIAN),
//                new Point(11,-25,Point.CARTESIAN),
//                new Point(20,-25,Point.CARTESIAN),
//                new Point(22,-25,Point.CARTESIAN),
//                new Point(22,-20,Point.CARTESIAN),
////                new Point(22,-16,Point.CARTESIAN),
////                new Point(22,-9,Point.CARTESIAN),
//                new Poi33nt(22,12,Point.CARTESIAN),
//                new Point(22,16,Point.CARTESIAN),
////                new Point(25,15,Point.CARTESIAN),
//                new Point(31.15,14.5,Point.CARTESIAN)
                new Point(4.9,-25, Point.CARTESIAN),
                new Point(28,12,Point.CARTESIAN),
                new Point(31,15,Point.CARTESIAN)
        )
        );
        toDropZoneThree.setConstantHeadingInterpolation(0);

        toHumanPickFromDropThree= new Path(new BezierCurve(
//                new Point(30,11,Point.CARTESIAN),
//                new Point(25,15.5,Point.CARTESIAN),
//                new Point(17,10.6,Point.CARTESIAN),
//                new Point(11.5,5.78,Point.CARTESIAN),
//                new Point(9.3,3.7,Point.CARTESIAN),
//                new Point(6.9,0.6969,Point.CARTESIAN),
//                new Point(6.9,-25,Point.CARTESIAN),
//                new Point(7,-25,Point.CARTESIAN)
                new Point(31,15,Point.CARTESIAN),
                new Point(20,13,Point.CARTESIAN),
//                new Point(10,0,Point.CARTESIAN),
                new Point(10,-25,Point.CARTESIAN),
                new Point(4.5,-25,Point.CARTESIAN)
        )
        );
        toHumanPickFromDropThree.setConstantHeadingInterpolation(Math.toRadians(0));

        toDropZoneFour= new Path(new BezierCurve(
//                new Point(4,-25,Point.CARTESIAN),
//                new Point(11,-25,Point.CARTESIAN),
//                new Point(20,-25,Point.CARTESIAN),
//                new Point(22,-25,Point.CARTESIAN),
//                new Point(22,-20,Point.CARTESIAN),
////                new Point(22,-16,Point.CARTESIAN),
////                new Point(22,-9,Point.CARTESIAN),
//                new Point(22,12,Point.CARTESIAN),
//                new Point(22,16,Point.CARTESIAN),
////                new Point(25,15,Point.CARTESIAN),
//                new Point(31.15,13,Point.CARTESIAN)
                new Point(4.5,-25, Point.CARTESIAN),
                new Point(28,14,Point.CARTESIAN),
                new Point(30.85,13.5,Point.CARTESIAN)
        )
        );
        toDropZoneFour.setConstantHeadingInterpolation(0);

        toHumanPickFromDropFour= new Path(new BezierCurve(
//                new Point(30,11,Point.CARTESIAN),
//                new Point(25,15.5,Point.CARTESIAN),
//                new Point(17,10.6,Point.CARTESIAN),
//                new Point(11.5,5.78,Point.CARTESIAN),
//                new Point(9.3,3.7,Point.CARTESIAN),
//                new Point(6.9,0.6969,Point.CARTESIAN),
//                new Point(6.9,-25,Point.CARTESIAN),
//                new Point(7,-25,Point.CARTESIAN)
                new Point(30.85,13.5,Point.CARTESIAN),
                new Point(20,12,Point.CARTESIAN),
//                new Point(10,0,Point.CARTESIAN),
                new Point(10,-25,Point.CARTESIAN),
                new Point(5,-25,Point.CARTESIAN)
        )
        );
        toHumanPickFromDropFour.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-100));

        toDropZoneFive= new Path(new BezierLine(
                new Point(6.4,-19.95,Point.CARTESIAN),
                new Point(32,14.8,Point.CARTESIAN)
        )
        );
        toDropZoneFive.setConstantHeadingInterpolation(0);

//        follower.followPath(toDropPreload);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.addData("x: ", follower.getPose().getX());
        telemetryA.addData("y: ", follower.getPose().getY());
        telemetryA.update();
//        new stopMotors(false);
        CommandScheduler.getInstance().reset();
//        new dclawTgrabPosCMD();
//        new dpivotTransInitPosCMD();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new followPath(toDropPreload),
                        new WaitUntilCommand(busy),
                        new followPath(toSampleOne)
                )
        );
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
//    @Override
//    public void loop() {
//        follower.update();
//        if (!follower.isBusy() && follower.getCurrentPath()==toDropPreload) {
//            follower.followPath(toSampleOne);
//        }
//        if (!follower.isBusy() && follower.getCurrentPath()==toSampleOne) {
//            follower.followPath(toHumanOne);
//        }
////        if (!follower.isBusy()){
////            follower.followPath(toHumanOne);
////        }

    public void run(){


        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();
//        currentTime=System.nanoTime();
//        looptime = currentTime-previousTime;
//        previousTime= currentTime;
//        telemetry.addData("looptime",looptime);
////        telemetry.addData("is it busy", busy.getAsBoolean());
//        telemetry.update();
    }
}
