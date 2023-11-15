package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.teleop.TeleOpMain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "Main")
public class RedFrontstage extends LinearOpMode {

    protected CenterStageBot bot = new CenterStageBot();
    protected Mecanum drive = null;

    enum State{
        PROP_DETECTION,
        PLACING_BACKDROP_PIXEL,
        PLACING,
        DROPPING,
        PARKING
    }

    private State state = State.PROP_DETECTION;

    private ElapsedTime time = new ElapsedTime();

    private Pose2d[] SPIKES = {
            new Pose2d(-40, -36, Math.toRadians(90)),
            new Pose2d(-46, -42, Math.toRadians(90)),
            new Pose2d(-36, -40, Math.toRadians(0))
    };

    private final double SPIKE_WAIT = 0.1;
    private final double INTAKE_POWER = -0.7;

    private int propPos = 0;

    public double[] BACKDROPS = {
            -42.75,
            -30.75,
            -36
    };

    double placeTime = 0;
    double placeTime2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.init(hardwareMap,FieldConstants.RED_FRONTSTAGE_START);
        drive = bot.getDrive();

        TrajectorySequence prep = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, -48))
                .waitSeconds(SPIKE_WAIT)
                .build();

        TrajectorySequence next = drive.trajectorySequenceBuilder(prep.end())
                .lineToConstantHeading(new Vector2d(-48, -48))
                .waitSeconds(SPIKE_WAIT)
                .build();

        TrajectorySequence[] trajectories = {
                drive.trajectorySequenceBuilder(prep.end())
                        .lineToLinearHeading(SPIKES[0])
                        .addSpatialMarker(SPIKES[0].vec(), ()->{bot.intake.setPower(INTAKE_POWER);})
                        .waitSeconds(1)
                        .back(5)
                        .build(),
                drive.trajectorySequenceBuilder(prep.end())
                        .lineToLinearHeading(SPIKES[1])
                        .addSpatialMarker(SPIKES[1].vec(), ()->{bot.intake.setPower(INTAKE_POWER);})
                        .waitSeconds(1)
                        .back(5)
                        .addDisplacementMarker(()->{bot.intake.setPower(0);})
                        .build(),
                drive.trajectorySequenceBuilder(prep.end())
                        .lineToLinearHeading(SPIKES[2])
                        .addSpatialMarker(SPIKES[2].vec(), ()->{bot.intake.setPower(INTAKE_POWER);})
                        .waitSeconds(1)
                        .back(5)
                        .addDisplacementMarker(()->{bot.intake.setPower(0);})
                        .build()
        };

        waitForStart();

        time.reset();
        boolean parked = false;

        //Go off wall
        drive.followTrajectorySequence(prep);

        while(opModeIsActive()){

            if(!parked) {
                switch (state) {
                    case PROP_DETECTION: {
                        if(!(bot.getDist() <= 15 && bot.getDist() >= 10)){
                            propPos++;
                            drive.followTrajectorySequence(next);
                            if(!(bot.getDist() <= 10 && bot.getDist() >= 4)) propPos++;
                        }

                        drive.followTrajectorySequence(trajectories[propPos]);
                        bot.intake.setPower(0);

                        state = State.PLACING_BACKDROP_PIXEL;
                        break;
                    }
                    case PLACING_BACKDROP_PIXEL: {

/*
                        bot.claw.toggle();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(54, BACKDROPS[propPos], Math.toRadians(180)))
                                .build()
                        );
                        placeTime2 = time.milliseconds();
                        state = State.PLACING;

 */
                        break;
                    }
                    case PLACING: {
                        bot.moveSlideToPos(1200);
                        if (time.milliseconds() - placeTime2 >= 1200) {
                            bot.setArmPos(0.19);
                            state = State.DROPPING;
                        }
                        break;
                    }
                    case DROPPING: {
                        if (time.milliseconds() - placeTime2 >= 1600) {
                            bot.claw.toggle();
                            bot.retractSlide(200);
                            state = State.PARKING;
                        }
                        break;
                    }
                    case PARKING: {
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .forward(4)
                                .waitSeconds(0.1)
                                .strafeLeft(24)
                                .build()
                        );
                        parked = true;
                        break;
                    }
                }
            }

            bot.update(this.telemetry);

            telemetry.addData("dist", bot.getDist());
            telemetry.update();

        }

        bot.storePose();
        TeleOpMain.side = TeleOpMain.Side.RED;
    }

}
