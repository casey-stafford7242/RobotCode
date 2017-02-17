package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="9101_test: Iterative", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class NineOneNoOne extends OpMode
{
    DcMotor left;
    DcMotor right;
    Servo servoLeft;
    Servo servoRight;
    DcMotor intakeMotor;
    DcMotor rightLift;
    DcMotor leftLift;
    DcMotor flyWheel;
    DcMotor flyWheelTop;


    @Override
    public void init()
    {
        left = hardwareMap.dcMotor.get("left");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        right = hardwareMap.dcMotor.get("right");
        servoLeft = hardwareMap.servo.get("servoLeft");
        servoRight = hardwareMap.servo.get("servoRight");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
        flyWheel = hardwareMap.dcMotor.get("flyWheel");
        flyWheelTop = hardwareMap.dcMotor.get("flyWheelTop");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop()
    {
        //sets the drive motors power
        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);
        if(gamepad1.a == true)
        {
            servoLeft.setPosition(.5);
        }
        if(gamepad1.b == true)
        {
            servoLeft.setPosition(0);
        }
        if(gamepad1.x == true)
        {
            servoRight.setPosition(.5);
        }
        if(gamepad1.y == true)
        {
            servoRight.setPosition(0);
        }

        if(gamepad2.dpad_up)
        {
            intakeMotor.setPower(1);
        }
        else if (gamepad2.dpad_down)
        {
            intakeMotor.setPower(-1);
        }
        else if (gamepad2.dpad_up == false && gamepad2.dpad_down == false)
        {
            intakeMotor.setPower(0);
        }

        if(gamepad2.left_bumper == true)
        {
            flyWheel.setPower(1);
            flyWheelTop.setPower(1);
        }
        if(gamepad2.right_bumper == true)
        {
            flyWheel.setPower(0);
            flyWheelTop.setPower(0);
        }


        // code that powers the Lift
        if (gamepad1.left_trigger > 0)
        {
            rightLift.setPower(gamepad1.left_trigger);
            leftLift.setPower(gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger > 0)
        {
            rightLift.setPower(-gamepad1.right_trigger);
            leftLift.setPower(-gamepad1.right_trigger);
        }
        else
        {
            rightLift.setPower(0);
            leftLift.setPower(0);
        }

        telemetry.addData("wooooooooo ", "hoooooooooo!!!");
    }
}















































//Haha Jk