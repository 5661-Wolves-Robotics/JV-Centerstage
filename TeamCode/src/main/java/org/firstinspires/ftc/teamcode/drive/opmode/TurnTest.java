package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    private PIDFController headingController = new PIDFController(Mecanum.HEADING_PID);

    private ElapsedTime time = new ElapsedTime();

    private int t = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drive = new Mecanum(hardwareMap);

        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setTargetPosition(Math.PI / 2);

        waitForStart();

        time.reset();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            if(time.milliseconds() - t >= 2000){
                headingController.setTargetPosition(Math.PI / 2);
            }
            if(time.milliseconds() - t >= 4000){
                headingController.setTargetPosition(-Math.PI / 2);
                t += 4000;
            }

            drive.setWeightedDrivePower(new Pose2d(
                    0, 0, (headingController.update(drive.getPoseEstimate().getHeading()) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH
            ));

            drive.update();
        }
    }
}
