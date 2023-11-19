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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(group = "Main")
public class BlueFrontstage extends LinearOpMode {

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
            new Pose2d(-39, 34, Math.toRadians(270)),
            new Pose2d(-47, 42, Math.toRadians(270)),
            new Pose2d(-32, 36, Math.toRadians(0))
    };

    private final double SPIKE_WAIT = 0.1;
    private final double INTAKE_POWER = -0.7;

    private int propPos = 0;

    public double[] BACKDROPS = {
            36,
            29,
            40
    };

    double placeTime = 0;

    double yAvg = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.init(hardwareMap, FieldConstants.BLUE_FRONTSTAGE_START, new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        drive = bot.getDrive();

        TrajectorySequence prep = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(-36, 44))
                .waitSeconds(SPIKE_WAIT)
                .build();

        TrajectorySequence next = drive.trajectorySequenceBuilder(prep.end())
                .lineToConstantHeading(new Vector2d(-48, 48))
                .waitSeconds(SPIKE_WAIT)
                .build();

        TrajectorySequence[] trajectories = new TrajectorySequence[3];

        for(int i = 0; i < 2; i++){
            trajectories[i] = drive.trajectorySequenceBuilder(prep.end())
                    .lineToLinearHeading(SPIKES[i])
                    .addTemporalMarker(2.0, ()->{bot.intake.setPower(INTAKE_POWER);})
                    .waitSeconds(1)
                    .back(7)
                    .build();
        }

        trajectories[2] = drive.trajectorySequenceBuilder(prep.end())
                .setReversed(true)
                .splineToLinearHeading(SPIKES[2], 0)
                .setReversed(false)
                .addTemporalMarker(1.6, ()->{bot.intake.setPower(INTAKE_POWER);})
                .waitSeconds(1)
                .back(7)
                .build();

        TrajectorySequence[] toJoin = {
                drive.trajectorySequenceBuilder(trajectories[0].end())
                        .turn(-Math.toRadians(90))
                        .splineTo(new Vector2d(-52, 18), Math.toRadians(270))
                        .splineTo(new Vector2d(44, 12), 0)
                        .build(),
                drive.trajectorySequenceBuilder(trajectories[1].end())
                        .strafeLeft(11)
                        .forward(30)
                        .splineTo(new Vector2d(44, 12), 0)
                        .build(),
                drive.trajectorySequenceBuilder(trajectories[2].end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, 0), 0)
                        .splineTo(new Vector2d(44, 12), 0)
                        .build()
        };

        waitForStart();
        boolean parked = false;

        //Go off wall
        drive.followTrajectorySequence(prep);

        double dist = 0;
        boolean detected = false;

        time.reset();

        while(opModeIsActive()){

            if(!parked) {
                switch (state) {
                    case PROP_DETECTION: {
                        switch(propPos){
                            case 0: {
                                if (bot.getDist() <= 11 && bot.getDist() >= 9) {
                                    detected = true;
                                    break;
                                }
                                if (time.milliseconds() >= 1200) {
                                    drive.followTrajectorySequence(next);
                                    time.reset();
                                    propPos++;
                                }
                                break;
                            }
                            case 1:{
                                if(bot.getDist() <= 10 && bot.getDist() >= 4){
                                    detected = true;
                                    break;
                                }
                                if(time.milliseconds() >= 1200){
                                    time.reset();
                                    propPos++;
                                    detected = true;
                                }
                                break;
                            }
                        }

                        if(detected) {
                            drive.followTrajectorySequence(trajectories[propPos]);
                            bot.intake.setPower(0);

                            state = State.PLACING_BACKDROP_PIXEL;
                        }
                        break;
                    }
                    case PLACING_BACKDROP_PIXEL: {
                        bot.claw.toggle();

                        drive.followTrajectorySequence(toJoin[propPos]);
                        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(46, 36, Math.toRadians(180)))
                                .waitSeconds(0.2)
                                .build()
                        );

                        List<AprilTagDetection> aprilTags = bot.getAprilTags();

                        int num = 0;
                        yAvg = 0;
                        for(AprilTagDetection tag : aprilTags){
                            if(tag.metadata != null) {
                                num++;
                                yAvg += tag.metadata.fieldPosition.get(1) + tag.ftcPose.x;
                            }
                        }
                        if(num != 0) {
                            yAvg /= num;
                            double yDiff = 36 - yAvg;

                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(55, BACKDROPS[propPos] + yDiff, Math.toRadians(180)), Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build()
                            );
                        } else {
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(55, BACKDROPS[propPos], Math.toRadians(180)), Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build()
                            );
                        }


                        bot.setSlidePos(1200);

                        state = State.PLACING;
                        break;
                    }
                    case PLACING: {

                        if (!bot.isSlideBusy()) {
                            bot.setArmState(CenterStageBot.ArmState.LOWERED);
                            state = State.DROPPING;
                            placeTime = time.milliseconds();
                        }

                        break;
                    }
                    case DROPPING: {
                        if (time.milliseconds() - placeTime >= 700) {
                            bot.claw.toggle();
                            bot.retractSlide(200);
                            state = State.PARKING;
                        }
                        break;
                    }
                    case PARKING: {
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .splineToConstantHeading(new Vector2d(46, 8), Math.toRadians(180))
                                .build()
                        );
                        parked = true;
                        break;
                    }
                }
            }

            telemetry.addData("yAvg", yAvg);

            telemetry.addData("dist", bot.getDist());
            telemetry.update();

            bot.update();

        }

        bot.storePose();
        FieldConstants.side = FieldConstants.Side.BLUE;
    }

}
