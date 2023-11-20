package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
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

    public static PIDCoefficients heading = new PIDCoefficients(5, 0.02, 0.3);
    private PIDFController headingController = new PIDFController(heading);

    private final double X_RATIO = 0.2988584907 * (0.9 / DriveConstants.WHEEL_RADIUS);
    private double slidePos = 0;

    enum State {
        PLACING,
        DRIVING
    }

    private State currState = State.DRIVING;

    private Pose2d driveInput = new Pose2d(0, 0, 0);

    private ElapsedTime time = new ElapsedTime();
    private double lastTime = 0;
    private double deltaTime = 0;

    private boolean colliding = true;

    @Override
    public void runOpMode() throws InterruptedException {

        bot.init(hardwareMap, new Pose2d(48, 0, Math.toRadians(180)), new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        Mecanum drive = bot.getDrive();

        driver = new Gamepad(gamepad1);
        placer = new Gamepad(gamepad2);

        bindButtons();

        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setTargetPosition(Math.toRadians(180));

        waitForStart();

        time.reset();

        while(opModeIsActive()){

            driver.update();
            placer.update();

            //DRIVER-----------

            switch(currState) {
                case DRIVING: {

                    driveInput = new Pose2d(
                            -driver.l_stick_y,
                            -driver.l_stick_x,
                            -driver.r_stick_x
                    );

                    bot.setSlidePower(placer.r_trigger - placer.l_trigger);

                    Pose2d driveEstimate = drive.getPoseEstimate();
                    Vector2d inputEstimate = driveInput.vec().times(deltaTime).times(DriveConstants.MAX_VEL).rotated(driveEstimate.getHeading());
                    Box robotBounds = new Box(new Pose2d(driveEstimate.vec().plus(inputEstimate), driveEstimate.getHeading()), 18, 18);

                    Vector2d collisionVec = FieldConstants.collision.checkBox(robotBounds);

                    if(collisionVec != null && colliding) {
                        double heading = drive.getRawExternalHeading();
                        Vector2d input = driveInput.vec().rotated(heading);

                        Vector2d collisionNorm = new Vector2d(-Math.abs(collisionVec.getX()), collisionVec.getY());

                        driveInput = new Pose2d(
                                collisionNorm.div(collisionNorm.norm()).times(input.norm()).plus(input).rotated(-heading),
                                driveInput.getHeading()
                        );
                        telemetry.addData("Collision", "X: %.2f | Y: %.2f", collisionVec.getX(), collisionVec.getY());
                    }

                    if(!drive.isBusy()) {
                        drive.setWeightedDrivePower(driveInput);
                    }

                    break;
                }
                case PLACING: {

                    bot.setSlidePower(placer.r_trigger - placer.l_trigger);

                    Vector2d placerDrive = null;
                    if(FieldConstants.side == FieldConstants.Side.RED) {
                        placerDrive = new Vector2d(
                                placer.l_stick_y * 0.2,
                                placer.l_stick_x * 0.2
                        );
                    } else {
                        placerDrive = new Vector2d(
                                placer.l_stick_y * 0.2,
                                placer.l_stick_x * 0.2
                        );
                    }

                    driveInput = new Pose2d(
                            placerDrive,
                            (headingController.update(drive.getPoseEstimate().getHeading()) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH
                    );

                    if(!drive.isBusy()) {
                        drive.setWeightedDrivePower(driveInput);
                    }

                    break;
                }
            }

            //UPDATE------------------

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("pos", bot.getSlidePos());

            bot.update();

            double newTime = time.seconds();
            deltaTime = newTime - lastTime;
            lastTime = newTime;
        }
    }

    private void bindButtons(){
        //DRIVER BUTTONS
        driver.bind(Gamepad.Button.X, Gamepad.ButtonState.PRESSED, ()->bot.toggleIntake());
        driver.bind(Gamepad.Button.B, Gamepad.ButtonState.PRESSED, ()->bot.intake.setPower(-1.0f));
        driver.bind(Gamepad.Button.B, Gamepad.ButtonState.RELEASED, ()->{
            if(bot.isIntakeDropped()) bot.intake.setPower(CenterStageBot.INTAKE_POWER);
            else bot.intake.setPower(0.0);
        });
        driver.bind(Gamepad.Button.A, Gamepad.ButtonState.PRESSED, ()->{
            currState = State.PLACING;
            placer.rumble(RUMBLE_DURATION);
            bot.claw.toggle();
            if(bot.isIntakeDropped()) bot.toggleIntake();
        });
        driver.bind(Gamepad.Button.R_BUMPER, Gamepad.ButtonState.PRESSED, ()->{colliding = !colliding;});

        //PLACER BUTTONS

        placer.bind(Gamepad.Button.Y, Gamepad.ButtonState.PRESSED, ()->bot.launchPlane());
        placer.bind(Gamepad.Button.A, Gamepad.ButtonState.PRESSED, ()->{
            bot.claw.toggle();
            if(currState == State.PLACING){
                bot.retractSlide(150);

                driver.rumble(RUMBLE_DURATION);
                currState = State.DRIVING;
            }
        });
        placer.bind(Gamepad.Button.DPAD_UP, Gamepad.ButtonState.PRESSED, ()->{
            if(bot.getArmState() == CenterStageBot.ArmState.LOWERED) bot.setArmState(CenterStageBot.ArmState.STORED);
            else bot.setArmState(CenterStageBot.ArmState.RAISED);
        });
        placer.bind(Gamepad.Button.DPAD_DOWN, Gamepad.ButtonState.PRESSED, ()->{
            if(bot.getArmState() == CenterStageBot.ArmState.RAISED) bot.setArmState(CenterStageBot.ArmState.STORED);
            else bot.setArmState(CenterStageBot.ArmState.LOWERED);
        });
    }

}
