package opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.subsystems.liftArmSlide;


@Autonomous(name = "testAuto")
public class testAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private config.subsystems.liftArmSlide slides;

    private Pose startPos = new Pose(0,0, Math.toRadians(0));
    private Pose interPos = new Pose(24, -24, Math.toRadians(90));
    private Pose endPos = new Pose(24, 24, Math.toRadians(45));

    private PathChain path;

    public void buildPaths() {
        path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(interPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), interPos.getHeading())
                .addPath(new BezierLine(new Point(interPos), new Point(endPos)))
                .setLinearHeadingInterpolation(interPos.getHeading(), endPos.getHeading())
                .addPath(new BezierLine(new Point(endPos), new Point(startPos)))
                .setLinearHeadingInterpolation(endPos.getHeading(), startPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(path, true);
                    slides.setSlideTarget(300);
                    slides.setPivotTarget(90);
                    setPathState(-1);
                }
                break;
            default:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    slides.setSlideTarget(50);
                    slides.setPivotTarget(45);
                }
                break;
        }

        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }
        @Override
        public void loop() {
            follower.update();
            // slides.update();
            telemetry.update();

            autonomousPathUpdate();
        }

        @Override
        public void init() {
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            pathTimer = new Timer();

            follower = new Follower(hardwareMap);
            slides = new config.subsystems.liftArmSlide(hardwareMap, telemetry, true);
            follower.setStartingPose(startPos);
            buildPaths();

        }

        @Override
        public void start() {
            pathTimer.resetTimer();
            setPathState(0);
        }

    }
    }