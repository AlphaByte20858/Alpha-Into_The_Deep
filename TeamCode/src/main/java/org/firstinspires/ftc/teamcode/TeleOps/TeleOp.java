package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOperado Dox Cria", group = "OpMode")
public class TeleOp extends OpMode {
    DcMotorEx MET, MEF, MDT, MDF, LSi, LSii, braço, roboAng; //Define os motores no sistema
    Servo yawC, garrai, garraii; //Define o nome dos servos no sistema
    boolean yawG, raw;
    ElapsedTime f = new ElapsedTime(); //define contador de tempo para as funções
    ElapsedTime tempo = new ElapsedTime(); // define  o contador do tempo decorrido para o PID
    double axial, lateral,yaw;

    IMU imu;
    double drive, turn, giro, angle;
    //Função de init
    public void init(){
        //Definição dos motores e servos para o Drive Hub
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");
        yawC = hardwareMap.get(Servo.class, "yawC");
        garrai = hardwareMap.get(Servo.class, "garrai");
        garraii = hardwareMap.get(Servo.class, "garraii");
        braço = hardwareMap.get(DcMotorEx.class, "braço");
        roboAng = hardwareMap.get(DcMotorEx.class, "roboAng");

        //Define a direção dos motores
        MDT.setDirection(DcMotorSimple.Direction.REVERSE);
        MDF.setDirection(DcMotorSimple.Direction.REVERSE);
        LSi.setDirection(DcMotorSimple.Direction.REVERSE);
        braço.setDirection(DcMotorSimple.Direction.REVERSE);
        roboAng.setDirection(DcMotorSimple.Direction.REVERSE);
        garraii.setDirection(Servo.Direction.REVERSE);

        //Redefine os valores dos motores
        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Define o metodo a ser usado de contabilização dos motores
        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        braço.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //define o meio de "freio" para os motores
        LSi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSii.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        braço.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationrobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
        imu.initialize(new IMU.Parameters(orientationrobot));
        //pré-definições antes de iniciar as funções
        yawC.setPosition(0);
        garrai.setPosition(0.27);
        garraii.setPosition(0.28);
        yawG = false;
        raw = true;
    }

    //funções que vão se repetir, utilizadas para a partida em si e contêm os sistemas
    public void loop(){
        movi(); //função da movimentação
        linear(); //função do sistema linear
        arm(); //função do braço
        Crvos(); // função dos servos
        telemetry.update();
    }

    public void movi(){
        //cálculos usados para obter os valores para as mecanum
        axial   = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.9;
        lateral = gamepad1.left_stick_x * 0.8;
        yaw     =  gamepad1.right_stick_x * 0.6;

        fieldOriented(axial, lateral);

        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        if(gamepad1.right_bumper){
            MotorsPower(motorEsquerdoFf * 0.5, motorDireitoFf * 0.5, motorEsquerdoTf * 0.5, motorDireitoTf * 0.5);
        }
        else {
            MotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
        }
    }
    public void MotorsPower(double p1, double p2, double p3,double p4) {
        MEF.setPower(p1);
        MDF.setPower(p2);
        MET.setPower(p3);
        MDT.setPower(p4);
    }

    public void Crvos(){
        //Angulação da garra
        if (gamepad2.y && f.seconds() >= 0.3){
            if (!yawG){
                yawC.setPosition(0.37);
                yawG = true;
            }
            else if (yawG == true){
                yawC.setPosition(0);
                yawG = false;
            }
            f.reset();
        }

        //Abrir/fechar a garra
        if (gamepad2.x && f.seconds() >= 0.5){
            if (raw == true){
                garrai.setPosition(0.29);
                garraii.setPosition(0.30);
                raw = false;
            }
            else if (!raw) {
                garrai.setPosition(0);
                garraii.setPosition(0);
                raw = true;
            }
            f.reset();
        }
    }

    public void linear(){
        //funções do linear
        LSi.setPower(gamepad2.right_trigger - gamepad2.left_trigger * 0.7);
        LSii.setPower(gamepad2.right_trigger - gamepad2.left_trigger * 0.7);

        //motor que angula o robô para o climb
        if (gamepad2.dpad_up){
            roboAng.setPower(1);
        }
        else if (gamepad2.dpad_down) {
            roboAng.setPower(-1);
        }
        else {
            roboAng.setPower(0);
        }

        telemetry.addData("Linear", LSi.getCurrentPosition());
    }

    int posAlvo;
    public void arm(){
        //define a força do braço
        if (gamepad2.right_bumper){
            /*
            posAlvo = 40;
            braço.setTargetPosition(posAlvo);
            braço.setPower(0.6);
            braço.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            REATIVAR DEPOIS DE VER AS POSIÇÕES
            */
            braço.setPower(1);
        }
        else if (gamepad2.left_bumper){
            braço.setPower(-1);
        }
        else {
            braço.setPower(0);
        }
        telemetry.addData("braço", braço.getCurrentPosition());
    }

    public void fieldOriented(double driveP,  double turnP){
        angle = gyroCalclate();
        axial = driveP * Math.cos(angle) - turnP * Math.sin(angle);
        lateral = driveP * Math.sin(angle) + turnP * Math.cos(angle);
    }

    private double gyroCalclate(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }
}