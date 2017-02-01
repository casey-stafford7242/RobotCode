package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Test: Iterative OpMode", group="Iterative OpMode")
@Disabled
public class TeleOpTest extends OpMode
{
    DcMotor motorLeft;
    DcMotor motorRight;
    public void init()
    {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop()
    {
        motorRight.setPower(gamepad1.right_stick_y);
        motorLeft.setPower(gamepad1.left_stick_y);
        motorLeft.getPower();
    }
}
