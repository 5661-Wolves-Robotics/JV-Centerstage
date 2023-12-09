package org.firstinspires.ftc.teamcode.util.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class LimitedMotor{

    private final DcMotorEx motor;
    private final int min, max;
    private boolean negative = false;

    public LimitedMotor(DcMotorEx motor, int min, int max)
    {
        this.motor = motor;
        this.min = min;
        this.max = max;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(max);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(0.0f);
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

    public void setTargetPosition(int pos)
    {
        motor.setTargetPosition(Range.clip(pos, min, max));
        motor.setPower(1);
    }

    public boolean isBusy(){
        return motor.isBusy();
    }

}
