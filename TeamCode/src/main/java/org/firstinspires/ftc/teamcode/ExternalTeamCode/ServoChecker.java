package org.firstinspires.ftc.teamcode.ExternalTeamCode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ServoChecker", group = "ServoTestClass")
public class ServoChecker extends OpMode
{
    Servo rightButtonPushServo;
    public void init()
    {
        rightButtonPushServo = hardwareMap.servo.get("rightButtonPushServo");
    }

    public void loop()
    {
        continuousServoPositionChecker(rightButtonPushServo);
    }

    public double continuousServoPositionChecker(Servo servo)
    {
        double firstPosition = 0;
        double secondPosition = 0;
        if(firstPosition == 0 && secondPosition == 0)
        {
            firstPosition = servo.getPosition();
            DbgLog.msg(String.valueOf(firstPosition));
            sleep(200);
            secondPosition = servo.getPosition();
            DbgLog.msg(String.valueOf(secondPosition));
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
