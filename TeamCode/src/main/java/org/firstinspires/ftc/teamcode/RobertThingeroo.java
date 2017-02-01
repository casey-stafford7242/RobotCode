package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Robert: ", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class RobertThingeroo extends OpMode
///////////////////////////////////////////////////////////////////////////////////////////////////
//I am the king of code
{

    DcMotor right;
    DcMotor flyWheel;
    DcMotor left;
    Servo leftServo;
    Servo rightServo;


    @Override
    public void init()
    {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        flyWheel = hardwareMap.dcMotor.get("flyWheel");
    }

    @Override
    public void loop()
    {
        right.setPower(gamepad1.right_stick_y);
        left.setPower(gamepad1.left_stick_y);
        if(gamepad1.a)
        {
            leftServo.setPosition(1);
            rightServo.setPosition(0);
        }
        if(gamepad1.b)
        {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////