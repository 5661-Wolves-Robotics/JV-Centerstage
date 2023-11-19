package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.teleop.TeleOpMain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "Main")
public class BlueBackstage extends LinearOpMode {

    protected CenterStageBot bot = new CenterStageBot();
    protected Mecanum drive = null;

    enum State{
        PROP_DETECTION,
        PLACING_SPIKE_PIXEL,
        PLACING_BACKDROP_PIXEL,
        PLACING,
        DROPPING,
        PARKING
    }

    private State state = State.PROP_DETECTION;

    private ElapsedTime time = new ElapsedTime();

    public Pose2d[] SPIKES = {
            new Pose2d(32, 34, Math.toRadians(180)),
            new Pose2d(11, 37, Math.toRadians(180)),
            new Pose2d(24, 26, Math.toRadians(180))
    };

    public double[] BACKDROPS = {
            43,
            29,
            36
    };

    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.init(hardwareMap, FieldConstants.BLUE_BACKSTAGE_START, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        drive = bot.getDrive();

        TrajectorySequence prepProp = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(8)
                .splineToLinearHeading(new Pose2d(36, 32, 0), Math.toRadians(180))
                .build();

        waitForStart();

        time.reset();
        double placeTime = 0;
        double placeTime2 = 0;
        int propPos = 0;
        boolean parked = false;

        //Go off wall
        drive.followTrajectorySequence(prepProp);

        while(opModeIsActive()){

            if(!parked) {
                switch (state) {
                    case PROP_DETECTION: {
                        double dist = bot.getDist();
                        if (dist >= 2 && dist <= 5) {
                            propPos = 0;
                            telemetry.addLine("Prop at 0");
                        } else if (dist >= 16 && dist <= 20) {
                            propPos = 1;
                            telemetry.addLine("Prop at 1");
                        } else {
                            propPos = 2;
                            telemetry.addLine("Prop at 2");
                        }

                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(SPIKES[propPos])
                                .build()
                        );

                        bot.intake.setPower(-0.6f);
                        placeTime = time.milliseconds();
                        state = State.PLACING_SPIKE_PIXEL;
                        break;
                    }
                    case PLACING_SPIKE_PIXEL:
                        if (time.milliseconds() - placeTime >= 1000) {
                            bot.intake.setPower(0);
                            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .back(5)
                                    .build()
                            );
                            state = State.PLACING_BACKDROP_PIXEL;
                        }
                        break;
                    case PLACING_BACKDROP_PIXEL: {
                        bot.claw.toggle();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(54, BACKDROPS[propPos], Math.toRadians(180)))
                                .build()
                        );
                        placeTime2 = time.milliseconds();
                        state = State.PLACING;
                        break;
                    }
                    case PLACING: {
                        bot.setSlidePos(1300);
                        if (time.milliseconds() - placeTime2 >= 1200) {
                            bot.setArmState(CenterStageBot.ArmState.LOWERED);
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
                                .strafeRight(24)
                                .build()
                        );
                        parked = true;
                        break;
                    }
                }
            }

            telemetry.addData("dist", bot.getDist());
            telemetry.update();

            bot.update();

        }

        bot.storePose();
        FieldConstants.side = FieldConstants.Side.BLUE;
    }

}
