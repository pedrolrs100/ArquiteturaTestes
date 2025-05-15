package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDeDecisao.Controladora;
import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDeDecisao.Controladora5mais0_sugando;
import org.firstinspires.ftc.teamcode.robot.Sensors.Vision.Limelight;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.LinearVertical;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.SubsistemasInferiores;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.SubsistemasSuperiores;
import org.firstinspires.ftc.teamcode.common.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDeDecisao.OrdersManager;

import java.util.ArrayList;
import java.util.List;

public class V5 {
    // Attributes
    public MecanumDrive md;
    public  static double eixoX = 0, eixoY = -30;
    public Telemetry telemetry;
    public OrdersManager carteiro;
    public  static V5Modes v5Mode = V5Modes.SPECIMEN;
    List<Encoder> leftEncs =  new ArrayList<>(), rightEncs = new ArrayList<>();

    public SubsistemasInferiores intakeInferior;
    public SubsistemasSuperiores outtakeIntakeSuperior;
    public Limelight limelight;
    //public BHI260IMU imu;
    HardwareMap hardwaremap;
    //Orientation angles;
    //public BHI260IMU.Parameters parameters = new IMU.Parameters();
    public boolean teelop = false, risky = false,
            lastSample = false;
    public static double deposit_y = -44, deposit_x = -42;
    public double heading;
    public Controladora controladoraBASKET;
    public Controladora5mais0_sugando controladoraSPECIMEN;
    public V5(HardwareMap hardwareMap, Telemetry telemetry) {
        md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // -Controle de leituras- \\
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //angles =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //heading = angles.firstAngle;
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        //



        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.intakeInferior = new SubsistemasInferiores(hardwareMap, telemetry);
        this.outtakeIntakeSuperior = new SubsistemasSuperiores(hardwareMap, telemetry);
        carteiro = new OrdersManager(telemetry);
        this.limelight = new Limelight(hardwareMap);
        this.controladoraBASKET = new Controladora();
        this.controladoraSPECIMEN = new Controladora5mais0_sugando();
    }

    /*
    aqui preciso colocar as ações prontas do chassi (e toda lógica pra isso funcionar) além de ações que envolvem o v2
    todo: gotoBasket
    todo: goToSubmersible
    todo: goNearTheObservationZone
     */
    public double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }
    public Action sensorMovimentation() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                /*if (intakeOutake.garra.colorSensor.alpha() < 100) {

                    md.leftBack.setPower(0.3);
                    md.leftFront.setPower(0.3);
                    md.rightFront.setPower(0.3);
                    md.rightBack.setPower(0.3);

                    return true;
                }


                if (intakeOutake.garra.colorSensor.alpha() > 160 && intakeOutake.garra.colorSensor.alpha() < 260) {
                    md.leftBack.setPower(0);
                    md.leftFront.setPower(0);
                    md.rightFront.setPower(0);
                    md.rightBack.setPower(0);
                    return false;
                }*/
                return true;
            }
        };
    }
    public Action MoveOuttake(V5 robot){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .strafeToConstantHeading(new Vector2d(eixoX, eixoY))
                        .build()
        );
    }
    public Action MoveSpline(V5 robot){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .splineTo(new Vector2d(eixoX, eixoY),Math.toRadians(-90))
                        .build()
        );
    }

    public Action MoveIntake(V5 robot){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .strafeToConstantHeading(new Vector2d(38, -60))
                        .build()
        );
    }
    public void  runStatesSpecimen(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
        if(!LinearVertical.hang){
            outtakeIntakeSuperior.runStatesSpecimen(carteiro, runtime, robot, gamepad);
        }
        intakeInferior.runStatesSpecimen(carteiro, runtime, robot, gamepad);
    }

    public void runStatesSample(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
        if(!LinearVertical.hang){
            outtakeIntakeSuperior.runStatesSample(carteiro, runtime, robot, gamepad);
        }

        intakeInferior.runStatesSample(carteiro, runtime, robot, gamepad);

    }
    public void runStatesSampleAutonomo(OrdersManager carteiro, double runtime, V5 robot) {
        outtakeIntakeSuperior.runStatesSampleAutonomo(carteiro, runtime, robot);
        intakeInferior.runStatesSampleAutonomo(carteiro, runtime, robot);
        carteiro.checkIfCanRun(runtime);
        carteiro.runTeleopActions(runtime);
    }

    public void runStatesSampleV5eMEIO(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {

        outtakeIntakeSuperior.runStatesSample(carteiro, runtime, robot, gamepad);
        intakeInferior.runStatesSampleV5eMEIO(carteiro, runtime, robot, gamepad);

    }



    /*****************************************/
    /*********** Funções Antigas  ************/
    /*****************************************/
    public Action actionTransfer(){
         return new SequentialAction(
                 outtakeIntakeSuperior.actionGoReadyTransfer(),
                 intakeInferior.intakeSuccao.TransferPositionAlcapao(),
                 intakeInferior.actionIntakeIrPraInitial(),
                 outtakeIntakeSuperior.actionGoTransfer()
         );
    }
    public void transfer(OrdersManager carteiro, double runtime) {
        carteiro.addOrder(actionTransfer(), 0, "transfer", runtime);
    }
    public void goReadytoHang(OrdersManager carteiro, double runtime) {
        carteiro.addOrder(actionGoReadytoHang(), 0, "goReadyHang", runtime);
    }
    public void goReadytoHang2(OrdersManager carteiro, double runtime) {
        carteiro.addOrder(actionGoReadytoHang(), 0, "goReadyHang", runtime);
    }
    public void goToHang(OrdersManager carteiro, double runtime) {
        carteiro.addOrder(actionGotoHang(), 0, "goToHang", runtime);
    }
    public Action actionGoReadytoHang() {
        Action readyToHangAction = new SequentialAction(

                outtakeIntakeSuperior.braco.goToReadyHang(),
                outtakeIntakeSuperior.garraSuperior.goToReadyHang(),
                intakeInferior.actionIntakeIrPraInitial(),
                outtakeIntakeSuperior.linearVertical.goToReadyHang()

        );
        return readyToHangAction;
    }
    public Action actionGoReadytoHang2() {
        Action readyToHangAction = new SequentialAction(

                outtakeIntakeSuperior.braco.goToReadyHang(),
                outtakeIntakeSuperior.garraSuperior.goToReadyHang(),
                intakeInferior.actionIntakeIrPraInitial(),
                outtakeIntakeSuperior.linearVertical.goToReadyHang2()

        );
        return readyToHangAction;
    }
    public Action actionGotoHang() {
        LinearVertical.hang = true;
        Action readyToHangAction = new SequentialAction(
                outtakeIntakeSuperior.braco.goToHang(),
                outtakeIntakeSuperior.garraSuperior.goToHang(),
                intakeInferior.actionIntakeIrPraInitial()
                //outtakeIntakeSuperior.linearVertical.goToHang()
        );
        return readyToHangAction;
    }


    public void runStatesSpecimenAutonomo(OrdersManager carteiro, double runtime, V5 robot) {
        outtakeIntakeSuperior.runStatesSpecimenAutonomo(carteiro, runtime, robot);
        intakeInferior.runStatesSpecimenAutonomo(carteiro, runtime, robot);
        carteiro.checkIfCanRun(runtime);
        carteiro.runTeleopActions(runtime);
    }
}
