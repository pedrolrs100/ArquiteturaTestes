package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Sugar;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Controller.ControladoresDeDecisao.OrdersManager;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.V5;
import org.firstinspires.ftc.teamcode.robot.Sensors.SensorCor;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.SubsistemasInferiores;

import java.util.HashMap;
@Config
public class IntakeSuccao{
    private V5 robot;
    public SensorCor colorSensorSugar;
    public int  red,
                blue,
                green,
                alpha;
    public Servo angulacao;
    public static boolean monitor = false, toggle = false;
    private double delay = 0.25;
    double cooldown = 0;
    private double cooldownAberturaGarra = 0;
    public static double power_Sugador = 1.0, pontoAtiv = 0.9, powerMesmo = 0.55;
    public DcMotorEx sugador;
    public Servo alcapao;
    public static double posicaoReadyIntakeAlcapao = 0.55;
    public static double posicaoIntakeAlcapao = 0.625;

    public static double posicaoTransferAngulacao = 0.528, posicaoIntakeAgulacao = 0.370, posicaoEjetarAngulacao = 0.420;
    // pos antiga do sugar ang 0.362
    private HashMap<SugarAngulationStates , Double> mapAngulation = new HashMap<>();
    public  SugarAngulationStates sugarAngulationStates  = SugarAngulationStates.INITIAL;

    private HashMap<AlcapaoStates , Double> mapAlcapao = new HashMap<>();
    public AlcapaoStates alcapaoStates  = AlcapaoStates.TRASNFER;


    public IntakeSuccao(HardwareMap hardwareMap){
        IntakeSuccao.power_Sugador = powerMesmo;
        angulacao = hardwareMap.get(Servo.class, HardwareNames.angulacaoSugarServo);
        sugador = hardwareMap.get(DcMotorEx.class,HardwareNames.SugadorMotorInferior);
        alcapao = hardwareMap.get(Servo.class,HardwareNames.alcapaoSugarServo);
        colorSensorSugar = new SensorCor(hardwareMap, HardwareNames.colorSensor1);

        mapAngulation.put(SugarAngulationStates.INTAKE, posicaoIntakeAgulacao);
        mapAngulation.put(SugarAngulationStates.TRANSFER, posicaoTransferAngulacao);
        mapAngulation.put(SugarAngulationStates.INITIAL, posicaoTransferAngulacao);
        mapAngulation.put(SugarAngulationStates.READY_TOINTAKE, posicaoTransferAngulacao);
        mapAngulation.put(SugarAngulationStates.EJECTED, posicaoEjetarAngulacao);


        mapAlcapao.put(AlcapaoStates.TOTALOPEN, 0.0);
        mapAlcapao.put(AlcapaoStates.INTAKE_SAMPLE, posicaoIntakeAlcapao);
        mapAlcapao.put(AlcapaoStates.READY_TOINTAKE, posicaoReadyIntakeAlcapao);

        mapAlcapao.put(AlcapaoStates.INITIAL,0.834);
        mapAlcapao.put(AlcapaoStates.INTAKE_SPECIMEN,0.632);
        mapAlcapao.put(AlcapaoStates.READY_TOINTAKE_SPECIMEN,0.632);
        mapAlcapao.put(AlcapaoStates.TOTALCLOSE, 1.0);
        mapAlcapao.put(AlcapaoStates.TRASNFER, 1.0);
    }
    public Action verifyColorSensor(){
        return new Action() {
            V5 robot;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if(colorSensorSugar.getAlpha() > 1500){
                    sugador.setPower(-1);
                    return false;
                }
                return true;
            }
        };
    }


    //GotoReadyToIntakeSpecimen
    public Action GotoReadyToIntakeSample(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.READY_TOINTAKE;
            alcapaoStates = AlcapaoStates.INTAKE_SAMPLE;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));


        });
    }

    public Action GotoIntakeSample(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.INTAKE;
            alcapaoStates = AlcapaoStates.INTAKE_SAMPLE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));

        });
    }

    public Action GotoReadyToIntakeSpecimen(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.READY_TOINTAKE;
            alcapaoStates = AlcapaoStates.INTAKE_SAMPLE;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));


        });
    }
    public Action GotoIntakeSpecimen(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.INTAKE;
            alcapaoStates = AlcapaoStates.INTAKE_SAMPLE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));

        });
    }
    public Action GotoEjectSample(){
        return new InstantAction(()->{
            sugarAngulationStates  = SugarAngulationStates.EJECTED;
            alcapaoStates = AlcapaoStates.INTAKE_SAMPLE;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));

        });
    }

    public Action IntakeSugar(){
        return new InstantAction(()->{
            sugador.setPower(power_Sugador);
        });
    }
    public Action IntakeAjeitarSample(){
        return new InstantAction(()->{
            sugador.setPower(power_Sugador * 0.8);
        });
    }
    public Action IntakeRepelir(){
        return new InstantAction(()->{
            sugador.setPower(power_Sugador * -1);
        });
    }
    public Action IntakeParar(){
        return new InstantAction(()->{
            sugador.setPower(0);
        });
    }
    public Action GoToFinishIntake(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));


        });
    }
    public Action GoToExpulse(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));


        });
    }
    public Action totalCloseAlcapao(){
        return new InstantAction(()->{
            alcapaoStates = AlcapaoStates.TOTALCLOSE;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }


    public Action IntakeSamplePositionAlcapao(){
        return new InstantAction(()->{
            alcapaoStates = AlcapaoStates.INTAKE_SAMPLE;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action TransferPositionAlcapao(){
        return new InstantAction(()->{
            alcapaoStates = AlcapaoStates.TRASNFER;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action ReadytoIntakePositionAlcapao(){
        return new InstantAction(()->{
            alcapaoStates = AlcapaoStates.READY_TOINTAKE_SPECIMEN;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action TotalOpenPositionAlcapao(){
        return new InstantAction(() -> {
            alcapaoStates = AlcapaoStates.TOTALOPEN;
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));
        });
    }
    public Action GoToInitial(){
        return new InstantAction(()->{
            sugarAngulationStates = SugarAngulationStates.INITIAL;
            alcapaoStates = AlcapaoStates.TRASNFER;
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
            alcapao.setPosition(mapAlcapao.get(alcapaoStates));

        });
    }
    public Action GoToTransfer(){
        return new InstantAction(()->{

            alcapaoStates = AlcapaoStates.TRASNFER;
            sugarAngulationStates = SugarAngulationStates.TRANSFER;
            alcapao.setPosition(mapAlcapao.get(AlcapaoStates.TRASNFER));
            angulacao.setPosition(mapAngulation.get(sugarAngulationStates));
        });
    }
    public Action gerenciadorDoFechamentoDaAlcapaoNoTeleop(double runTime) {
        if(runTime < this.cooldownAberturaGarra) {
            return new InstantAction(() -> {});
        }
        this.cooldownAberturaGarra = runTime + this.delay;

        if (this.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE) {
            return this.TransferPositionAlcapao();
        }
        return this.IntakeSamplePositionAlcapao();

    }

    public void gerenciadorDoAlcapaoManual(OrdersManager carteiro, double runTime){
        if(runTime < cooldown){
            return;
        }
        if(alcapaoStates == AlcapaoStates.TOTALOPEN ){
            carteiro.addOrder(IntakeSamplePositionAlcapao(),0.0,"alcapao",runTime);
            SubsistemasInferiores.alcapaoManual = false;
            return;
        }
        if(alcapaoStates != AlcapaoStates.TOTALOPEN){
            carteiro.addOrder(TotalOpenPositionAlcapao(),0.0,"alcapao",runTime);
        }
        cooldown = runTime + 0.3; // 0.35

    }
    public void gerenciadorDoSugador(GamepadEx gamepad, double runTime){
        if(runTime < cooldown){
            return;
        }
        cooldown = runTime + 0.35;

        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= pontoAtiv){
            if(sugador.getPower() <= 0) {
                sugador.setPower(+power_Sugador);
                return;
            }
            if(sugador.getPower() > 0) {
                sugador.setPower(0);
                return;
            }
        }
        if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= pontoAtiv){

            if(sugador.getPower() >= 0) {
                sugador.setPower(-power_Sugador);
                return;
            }
            if(sugador.getPower() < 0) {
                sugador.setPower(0);
            }
        }
    }

    public void gerenciadorDoSugadorManual(GamepadEx gamepad, double runTime){
        if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0){
            sugador.setPower(-power_Sugador * gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)* 1.2);
        }
        else if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
            sugador.setPower(-power_Sugador *-gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) );
        }
        else if(sugarAngulationStates  != SugarAngulationStates.INTAKE ){
            sugador.setPower(0);
        }


    }


    public Action pwmDiseable(){
        return  new InstantAction(()->{
            angulacao.getController().pwmDisable();
        });
    }
    public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("======================================");
            telemetry.addLine("       TELEMETRIA DO INTAKE SUCÇÃO    ");
            telemetry.addLine("======================================");
            telemetry.addData("INTK ANGULAR getPosition",angulacao.getPosition());
            telemetry.addData("INTK ANGULAR PMW", angulacao.getController().getPwmStatus());
            telemetry.addData("INTK ALCAPAO getPosition", alcapao.getPosition());
            telemetry.addData("INTK ALCAPAO state", alcapaoStates);
            telemetry.addData("INTK ALCAPAO PMW",alcapao.getController().getPwmStatus());
            telemetry.addData("INTK ANGULAR State",sugarAngulationStates);
            telemetry.addData("INTK current", sugador.getCurrent(CurrentUnit.AMPS) + " AMPS");


            telemetry.addData("Alpha", colorSensorSugar.getAlpha());
            telemetry.addData("Red", colorSensorSugar.getRed());
            telemetry.addData("Blue",colorSensorSugar.getBlue());
            telemetry.addData("Green", colorSensorSugar.getGreen());
            telemetry.addData("Distancia", colorSensorSugar.getDistance());
        }}
}
