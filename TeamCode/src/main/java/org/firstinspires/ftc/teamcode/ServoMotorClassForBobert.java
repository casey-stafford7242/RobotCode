package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="servo: Iterative", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class ServoMotorClassForBobert extends OpMode
{
    Servo servo;

    @Override
    public void init()
    {
        servo = hardwareMap.servo.get("servo");
        servo.setPosition(.5);
    }

    @Override
    public void loop()
    {
        if(gamepad1.a)
        {
            servo.setPosition(1);
        }

        if(gamepad1.b)
        {
            servo.setPosition(0);
        }

    }

}