package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.motor.LimitedMotor;

public class DualLinearSlide extends SubsystemBase {

    private final LimitedMotor m_right, m_left;
    final int MAX_HEIGHT;

    public DualLinearSlide(HardwareMap hardwareMap, String right, String left, int maxHeight){
        m_right = new LimitedMotor(hardwareMap.get(DcMotorEx.class, right), 0, maxHeight);
        m_left = new LimitedMotor(hardwareMap.get(DcMotorEx.class, left), 0, maxHeight);

        m_left.setDirection(DcMotorSimple.Direction.REVERSE);

        m_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MAX_HEIGHT = maxHeight;
    }

    public void setPower(double pow){
        m_right.setPower(pow);
        m_left.setPower(pow);
    }

    public int getPosition(){
        return (int)((m_right.getPosition() + m_left.getPosition()) / 2.0);
    }

    public void setTargetPosition(int pos){
        m_right.setTargetPosition(pos);
        m_left.setTargetPosition(pos);
    }

    public boolean isBusy(){
        return m_left.isBusy() || m_right.isBusy();
    }

    public void moveToMin(){
        setTargetPosition(0);
    }
}
