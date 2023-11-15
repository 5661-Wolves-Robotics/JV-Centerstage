package org.firstinspires.ftc.teamcode.util.motor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class LimitedMotorImpl extends DcMotorImplEx{

    private int min, max;
    private boolean negative = false;

    public LimitedMotorImpl(DcMotorController controller, int portNumber)
    {
        this(controller, portNumber, Direction.FORWARD);
    }

    public LimitedMotorImpl(DcMotorController controller, int portNumber, Direction direction)
    {
        this(controller, portNumber, direction, MotorConfigurationType.getUnspecifiedMotorType());
    }

    public LimitedMotorImpl(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType)
    {
        super(controller, portNumber, direction, motorType);

    }

    @Override
    public synchronized void setPower(double power) {
        if(power < 0)
        {
            if(!negative){
                negative = true;
            }
            if(getTargetPosition() != min) setTargetPosition(min);
            super.setPower(-power);
        } else {
            if(negative){
                negative = false;
            }
            if(getTargetPosition() != max) setTargetPosition(max);
            super.setPower(power);
        }
    }

    @Override
    public synchronized void setDirection(Direction direction) {
        super.setDirection(direction);
        if(negative)
        {
            setTargetPosition(min);
        } else
        {
            setTargetPosition(max);
        }
    }
/*
    @Override
    public void setLimits(int min, int max) {
        this.min = min;
        this.max = max;

        setMode(STOP_AND_RESET_ENCODER);
        setMode(RUN_USING_ENCODER);
        setTargetPosition(max);
        setMode(RUN_TO_POSITION);

        setPower(0.0f);
    }

    @Override
    public void moveToMin(){
        setTargetPosition(min);
        setPower(1.0f);
    }

 */
}
