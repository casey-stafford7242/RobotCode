package org.firstinspires.ftc.teamcode.ExternalTeamCode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoChecker
{

    public double continuousServoPositionChecker(Servo servo)
    {
        double firstPosition = 0;
        double secondPosition = 0;
        if(firstPosition == 0 && secondPosition == 0)
        {
            firstPosition = servo.getPosition();
            sleep(200);
            secondPosition = servo.getPosition();
        }
        return (Math.abs(secondPosition - firstPosition));
    }

    public void sleep(long mills)
    {
        try{
            Thread.sleep(mills);
        }
        catch(InterruptedException e)
        {
            e.printStackTrace();
        }
    }

}
