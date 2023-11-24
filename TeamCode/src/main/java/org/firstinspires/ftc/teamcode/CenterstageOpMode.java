package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

public abstract class CenterstageOpMode extends LinearOpMode {

    protected CenterStageBot bot = new CenterStageBot();
    protected Mecanum drive = null;

    protected MultipleTelemetry telemetry = null;

    protected ElapsedTime time = new ElapsedTime();

    protected double lastTime = 0;
    protected double deltaTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        opModeInit();

        waitForStart();

        opModeStart();

        while(opModeIsActive()){
            opModeUpdate();
            bot.update();

            double newTime = time.seconds();
            deltaTime = newTime - lastTime;
            lastTime = newTime;
        }
    }

    public void opModeInit(Pose2d startPose){
        bot.init(hardwareMap, startPose, telemetry);
        drive = bot.getDrive();
    }

    public void opModeInit(){
        bot.init(hardwareMap, Robot.getStoredPose(), telemetry);
        drive = bot.getDrive();
    }

    public void opModeStart(){
        time.reset();
    }

    public void opModeUpdate(){}

}
