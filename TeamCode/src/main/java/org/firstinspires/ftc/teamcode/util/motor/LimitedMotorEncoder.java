package org.firstinspires.ftc.teamcode.util.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/*
public interface LimitedMotor extends DcMotorEx{

    @Override
    void setPower(double power);

    @Override
    void setDirection(Direction direction);

    void setLimits(int min, int max);

    void moveToMin();
}
*/

public class LimitedMotorEncoder{

    private DcMotorEx motor;
    private int min, max;
    private boolean negative = false;
    private int currentPosition = 0;


    public LimitedMotorEncoder(DcMotorEx motor, int min, int max)
    {
        this.motor = motor;
        this.min = min;
        this.max = max;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(min);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(1.0f);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public void setPower(double power)
    {
        if(power < 0)
        {
            if(!negative){
                negative = true;
            }
            if(motor.getTargetPosition() != min) motor.setTargetPosition(min);
            motor.setPower(-power);
        } else {
            if(negative){
                negative = false;
            }
            if(motor.getTargetPosition() != max) motor.setTargetPosition(max);
            motor.setPower(power);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction)
    {
        motor.setDirection(direction);
        if(negative)
        {
            motor.setTargetPosition(min);
        } else
        {
            motor.setTargetPosition(max);
        }
    }

    public double getPosition()
    {
        return ((double)(motor.getCurrentPosition() - min)) / (max - min);
    }

    public int getIntPosition()
    {
        return motor.getCurrentPosition();
    }

    public boolean getTargetSide()
    {
        return negative;
    }

    public void setPosition(int pos)
    {
        motor.setTargetPosition(Range.clip(pos, min, max));
    }

    public void moveToMin(){
        motor.setTargetPosition(min);
        motor.setPower(1.0f);
    }

    public void moveToPosition(int pos){
        motor.setTargetPosition(pos);
        motor.setPower(1.0f);
    }

    public boolean isBusy(){
        return motor.isBusy();
    }

}
