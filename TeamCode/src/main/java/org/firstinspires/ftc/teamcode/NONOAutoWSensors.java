package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;


@Autonomous(name="NONOAuto: Iterative", group="Iterative Opmode")  // @TeleOp(...) is the other common choice
//@Disabled
public class NONOAutoWSensors extends OpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor leftFlyWheel;
    DcMotor rightFlyWheel;
    DcMotor leftLift;
    DcMotor rightLift;


// C++ = betterThanJava;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    @Override
    public void loop()
    {

    }

    


    public void runUsingSensors()
    {

    }
}