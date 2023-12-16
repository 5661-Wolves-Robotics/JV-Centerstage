package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.motor.LimitedMotor;

public class DualLinearSlide extends SubsystemBase {

    private final DcMotorEx m_right, m_left;
    final int MAX_ROT;
    private final double TPR = 537.7;
    private final double RPM = 312;

    private double targetPosition = 0;

    private final ElapsedTime time = new ElapsedTime();
    private double deltaTime = 0;

    public DualLinearSlide(HardwareMap hardwareMap, String right, String left, int maxRot){
        m_right = hardwareMap.get(DcMotorEx.class, right);
        m_left = hardwareMap.get(DcMotorEx.class, left);

        m_left.setDirection(DcMotorSimple.Direction.REVERSE);

        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_right.setTargetPosition((int)targetPosition);
        m_left.setTargetPosition((int)targetPosition);

        m_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MAX_ROT = maxRot;

        m_right.setPower(1);
        m_left.setPower(1);

        time.reset();
    }

    public void setPower(double pow){
        targetPosition = Range.clip(targetPosition + ((RPM / 60) * TPR) * deltaTime * pow, 0, MAX_ROT);
    }

    public int getPosition(){
        return (int)((m_right.getCurrentPosition() + m_left.getCurrentPosition()) / 2.0);
    }

    public void setTargetPosition(int pos){
        targetPosition = pos;
    }

    public boolean isBusy(){
        return m_left.isBusy() || m_right.isBusy();
    }

    private double lastTime = time.seconds();

    @Override
    public void periodic() {
        m_right.setTargetPosition((int)targetPosition);
        m_left.setTargetPosition((int)targetPosition);

        double newTime = time.seconds();
        deltaTime = newTime - lastTime;
        lastTime = newTime;
    }
}
