package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SpearedVoidSample extends OpMode {
    DcMotorEx LS, InTK;
    public void init() {
        LS = hardwareMap.get(DcMotorEx.class, "LS");
        InTK = hardwareMap.get(DcMotorEx.class, "InTK");

        LS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        InTK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        InTK.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop() {
        LS.setPower((gamepad1.right_trigger - gamepad1.left_trigger) * 0.8);
        InTK.setPower((gamepad2.right_trigger - gamepad2.left_trigger));

        telemetry.addData("Sistema Linear:", LS.getPower());
        telemetry.addData("In Take:", InTK.getPower());
    }
}
