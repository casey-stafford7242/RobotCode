package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;


@TeleOp(name="Two Motor Test: Iterative Opmode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class TwoMotorTestMcTesterson extends OpMode
{
    DcMotor motorRight, motorLeft;
    @Override
    public void init()
    {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
    }

    @Override
    public void loop()
    {
        motorLeft.setPower(gamepad1.left_stick_y);
        motorRight.setPower(gamepad1.right_stick_y);
    }


}
