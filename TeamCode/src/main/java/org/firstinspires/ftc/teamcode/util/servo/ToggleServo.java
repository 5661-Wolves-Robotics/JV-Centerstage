package org.firstinspires.ftc.teamcode.util.servo;

import com.qualcomm.robotcore.hardware.Servo;

public class ToggleServo {

    private float truePos;
    private float falsePos;
    private Servo servo;
    private boolean state;

    public ToggleServo(Servo servo, float truePos, float falsePos, boolean state, boolean init)
    {
        this.servo = servo;
        this.truePos = truePos;
        this.falsePos = falsePos;
        this.state = state;

        if(init) setState(state);
    }

    public void setState(boolean state)
    {
        if(state) servo.setPosition(truePos);
        else servo.setPosition(falsePos);
        this.state = state;
    }

    public boolean getState()
    {
        return state;
    }

    public void toggle()
    {
        state = !state;
        if(state) servo.setPosition(truePos);
        else servo.setPosition(falsePos);
    }

    public double getPosition()
    {
        return servo.getPosition();
    }

    public void setDirection(Servo.Direction direction)
    {
        servo.setDirection(direction);
    }

}
