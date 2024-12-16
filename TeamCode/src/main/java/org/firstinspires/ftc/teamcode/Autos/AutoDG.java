package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoDG", group = "LinearOpMode")
public class AutoDG extends LinearOpMode {
    public DcMotorEx MEF, MET, MDF, MDT, LSi, LSii, braço;

    public Servo garrai, garraii;

    ElapsedTime tempo = new ElapsedTime();

    @Override
    public void runOpMode() {
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");
        braço = hardwareMap.get(DcMotorEx.class, "braço");
        garrai = hardwareMap.get(Servo.class, "garrai");
        garraii = hardwareMap.get(Servo.class, "garraii");

        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");

        braço.setDirection(DcMotorSimple.Direction.REVERSE);
        garraii.setDirection(Servo.Direction.REVERSE);
        LSi.setDirection(DcMotorSimple.Direction.REVERSE);

        LSi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSii.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        modemoto(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotorEx.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MET.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotorEx.Direction.FORWARD);

        garrai.setPosition(0.36);
        garraii.setPosition(0.56);

        resetRuntime();
        waitForStart();

        if (opModeIsActive()) {
            allMotorsPower(-0.1, 0.1, 0.1, -0.1);
            esperar(1);
            allMotorsPower(0, 0, 0, 0);
            esperar(2);
            allMotorsPower(-0.05, -0.05, -0.05, -0.05);
            esperar(1);
            braço.setPower(0.7);
            allMotorsPower(0, 0, 0, 0);
            esperar(2);
            braço.setPower(0);
            linearPower(0.73);
            esperar(1);
            linearPower(0);
            esperar(2);
            braço.setPower(-0.3);
            esperar(1);
            braço.setPower(0);
            linearPower(-0.3);
            esperar(1);
           /* allMotorsPower(-0.5, -0.5, -0.5, -0.5);
            esperar(1);
            allMotorsPower(0, 0, 0, 0);
            sleep(3000);
            linearPower(-0.2);
            esperar(1);*/
            garrai.setPosition(0);
            garraii.setPosition(0);
            esperar(1);
            linearPower(0.03);
            esperar(0.5);
            allMotorsPower(0.08, -0.08, -0.08, 0.08);
            esperar(1);
            allMotorsPower(0.1, 0.1, 0.1, 0.1);


        }
    }

    public void esperar(double temp) {
        tempo.reset();
        while (tempo.seconds() < temp) {

        }
    }

    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT) {
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }

    public void linearPower(double paLS) {
        LSi.setPower(paLS);
        LSii.setPower(paLS);
    }

    public void modemoto(DcMotorEx.RunMode mode) {
        MDF.setMode(mode);
        MDT.setMode(mode);
        MEF.setMode(mode);
        MET.setMode(mode);
    }
}
