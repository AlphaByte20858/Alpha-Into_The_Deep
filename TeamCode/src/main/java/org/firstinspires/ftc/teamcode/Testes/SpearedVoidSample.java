package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SpearedVoidSample extends OpMode {
    DcMotorEx LSi, LSii;
    public void init() {
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");

        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LSi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LSii.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void loop() {
        LSi.setPower((gamepad1.right_trigger - gamepad1.left_trigger) * 0.7);
        LSii.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * 0.7);

        PID();

        telemetry.addData("Sistema Linear:", LSi.getPower());
        telemetry.addData("In Take:", LSii.getPower());
    }
    public void PID(){
        //continuar depois
        double error, lerror, p,i,d;
        error = LSi.getCurrentPosition();
    }
}