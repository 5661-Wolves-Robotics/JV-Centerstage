package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.util.Gamepad;
import org.firstinspires.ftc.teamcode.util.collision.shapes.Box;

@Config
@TeleOp(group = "Main")
public class TeleOpMain extends LinearOpMode {

    protected CenterStageBot bot = new CenterStageBot();

    private Gamepad driver = null;
    private Gamepad placer = null;
    public static int RUMBLE_DURATION = 250;
    public static enum Side {
        BLUE,
        RED
    }
    public static Side side = Side.RED;

    public static PIDCoefficients heading = new PIDCoefficients(5, 0.02, 0.3);
    private PIDFController headingController = new PIDFController(heading);

    private final double X_RATIO = 0.2988584907 * (0.9 / DriveConstants.WHEEL_RADIUS);
    private double slidePos = 0;

    enum State {
        PLACING,
        DRIVING
    }

    State currState = State.DRIVING;

    Pose2d driveInput = new Pose2d(0, 0, 0);

    ElapsedTime time = new ElapsedTime();
    double lastTime = 0;
    double deltaTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        bot.init(hardwareMap, Robot.getStoredPose());
        Mecanum drive = bot.getDrive();

        driver = new Gamepad(gamepad1);
        placer = new Gamepad(gamepad2);

        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setTargetPosition(Math.toRadians(180));

        waitForStart();

        time.reset();

        while(opModeIsActive()){

            driver.getInput();
            placer.getInput();

            //DRIVER-----------

            switch(currState) {
                case DRIVING: {

                    driveInput = new Pose2d(
                            -driver.l_stick_y,
                            -driver.l_stick_x,
                            -driver.r_stick_x
                    );

                    if (driver.getButtonState(Gamepad.Button.X) == Gamepad.ButtonState.PRESSED) {
                        bot.toggleIntake();
                    }

                    if (driver.getButtonState(Gamepad.Button.B) == Gamepad.ButtonState.PRESSED) {
                        bot.intake.setPower(-1.0);
                    } else if (driver.getButtonState(Gamepad.Button.B) == Gamepad.ButtonState.RELEASED) {
                        if(bot.isIntakeDropped()) bot.intake.setPower(0.8);
                        else bot.intake.setPower(0.0);
                    }

                    if (driver.getButtonState(Gamepad.Button.A) == Gamepad.ButtonState.PRESSED) {
                        currState = State.PLACING;

                        placer.rumble(RUMBLE_DURATION);

                        bot.claw.toggle();

                        if(bot.isIntakeDropped()) bot.toggleIntake();
                    }

                    if(placer.getButtonState(Gamepad.Button.Y) == Gamepad.ButtonState.PRESSED) {
                        bot.launchPlane();
                    }

                    if(placer.getButtonState(Gamepad.Button.X) == Gamepad.ButtonState.PRESSED) {
                        if(bot.getArmState() == CenterStageBot.ArmState.LOWERED) bot.setArmState(CenterStageBot.ArmState.STORED);
                        else bot.setArmState(CenterStageBot.ArmState.LOWERED);
                    }

                    if(placer.getButtonState(Gamepad.Button.B) == Gamepad.ButtonState.PRESSED) {
                        if(bot.getArmState() == CenterStageBot.ArmState.RAISED) bot.setArmState(CenterStageBot.ArmState.STORED);
                        else bot.setArmState(CenterStageBot.ArmState.RAISED);
                    }

                    if(placer.getButtonState(Gamepad.Button.A) == Gamepad.ButtonState.PRESSED) {
                        bot.claw.toggle();
                    }

                    bot.setSlidePower(placer.r_trigger - placer.l_trigger);

                    Pose2d driveEstimate = drive.getPoseEstimate();
                    Vector2d inputEstimate = driveInput.vec().times(deltaTime);
                    Box robotBounds = new Box(new Pose2d(driveEstimate.vec().plus(inputEstimate), driveEstimate.getHeading()), 18, 18);

                    Vector2d collisionVec = FieldConstants.collision.checkBox(robotBounds);

                    if(collisionVec != null) {
                        driveInput = driveInput.plus(new Pose2d(
                                collisionVec.div(collisionVec.norm()).times(driveInput.vec().norm()),
                                0
                        ));
                    }

                    if(!drive.isBusy()) {
                        drive.setWeightedDrivePower(driveInput);
                    }

                    break;
                }
                case PLACING: {

                    /*
                    double dpadY = (placer.isActive(Gamepad.Button.DPAD_UP) ? 1 : 0) + (placer.isActive(Gamepad.Button.DPAD_DOWN) ? -1 : 0);

                    bot.setSlidePower(dpadY);

                    driveInput = new Pose2d(
                            dpadY * X_RATIO,
                            placer.l_stick_x * 0.2,
                            (headingController.update(drive.getPoseEstimate().getHeading()) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH
                    );
                     */

                    bot.setSlidePower(placer.r_trigger - placer.l_trigger);

                    Vector2d placerDrive = new Vector2d(
                            placer.l_stick_y * 0.2,
                            placer.l_stick_x * 0.2
                    );

                    switch(side){
                        case RED:
                            placerDrive = placerDrive.rotated(Math.toRadians(90));
                            break;
                        case BLUE:
                            placerDrive = placerDrive.rotated(-Math.toRadians(90));
                            break;
                    }

                    driveInput = new Pose2d(
                            placerDrive,
                            (headingController.update(drive.getPoseEstimate().getHeading()) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH
                    );

                    if (placer.getButtonState(Gamepad.Button.A) == Gamepad.ButtonState.PRESSED) {
                        bot.claw.toggle();
                        bot.retractSlide(150);

                        driver.rumble(RUMBLE_DURATION);
                        currState = State.DRIVING;
                    }

                    if(placer.getButtonState(Gamepad.Button.X) == Gamepad.ButtonState.PRESSED) {
                        if(bot.getArmState() == CenterStageBot.ArmState.LOWERED) bot.setArmState(CenterStageBot.ArmState.STORED);
                        else bot.setArmState(CenterStageBot.ArmState.LOWERED);
                    }

                    if(!drive.isBusy()) {
                        drive.setFieldCentricWeightedDrivePower(driveInput);
                    }

                    break;
                }
            }

            //UPDATE------------------


            bot.update(this.telemetry);

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("pos", bot.getSlidePos());
            telemetry.addData("Dist", bot.getDist());

            telemetry.update();

            double newTime = time.seconds();
            deltaTime = newTime - lastTime;
            lastTime = newTime;
        }
    }

}
