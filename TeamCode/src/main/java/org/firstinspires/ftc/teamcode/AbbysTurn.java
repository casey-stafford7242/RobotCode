package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;


@TeleOp(name="91No1: Iterative", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class AbbysTurn extends OpMode
{
    DcMotor left;
    DcMotor right;
    DcMotor rightLift;
    DcMotor leftLift;
    DcMotor rightSpinnyWheel;
    DcMotor leftSpinnyWheel;

    @Override
    public void init()
    {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");
        rightSpinnyWheel = hardwareMap.dcMotor.get("rightSpinnyWheel");
        leftSpinnyWheel = hardwareMap.dcMotor.get("leftSpinnyWheel");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSpinnyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.left_stick_y);

        // Spinny Wheel code
        if (gamepad1.right_bumper == true)
        {
            leftSpinnyWheel.setPower(1);
            rightSpinnyWheel.setPower(1);
        }
        else if (gamepad1.left_bumper == true)
        {
            leftSpinnyWheel.setPower(-1);
            rightSpinnyWheel.setPower(-1);
        }
        else if (gamepad1.left_bumper == false && gamepad1.right_bumper == false)
        {
            leftSpinnyWheel.setPower(0);
            rightSpinnyWheel.setPower(0);
        }

        // Lift Code
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
        else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
        {
            rightLift.setPower(0);
            leftLift.setPower(0);
        }
    }
}
//Dade and Abby's newfound freindship!
//Is in hell where it belongs

//haha yteah me too
//the comments are the only imp0rotaent part of this <code></code>
//the comments are the only things working
















































//Haha Jk