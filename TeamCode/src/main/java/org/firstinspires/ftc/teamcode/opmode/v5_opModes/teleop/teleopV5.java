package org.firstinspires.ftc.teamcode.opmode.v5_opModes.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.ControlHubServer;
import org.firstinspires.ftc.teamcode.common.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Controller.OrdersManager;
import org.firstinspires.ftc.teamcode.robot.V5;
import org.firstinspires.ftc.teamcode.robot.V5Modes;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal.LinearHorizontalMotor;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Sugar.AlcapaoStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.LinearVertical;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.UpperSubsystemStates;


import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Teleoperado V5")
@Config

/****************************\
 Desligar os monitores
 \****************************/





public class teleopV5 extends OpMode {
    List<Servo> servos = new ArrayList<>(4);
    private V5 robot;
    private ControlHubServer server;
    boolean sugadorEstado= false ;

    GamepadEx gamepadEx1,gamepadEx2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static double powerDriveOut = -1, powerStrafeOut = 1,powerDriveInt  = 1,powerStrafeInt = -1;

    private double delayMode = 0.3, cooldownMode = 0, lastUpdateTime = 0, updateInterval = 0.050, loopTime = 0, lastTime = 0; // 50ms;



    public boolean needToOutake = false;

    @Override
    public  void init() {
        robot = this.createRobot(hardwareMap);
        Pose2d initialPose = new Pose2d(8.5, -61, Math.toRadians(-90));

        robot.md.pose = initialPose;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //JsonFileCreator.createJsonFile();

        //robot.imu.initialize(robot.parameters);

    }

    /*********************************************************************************************************************************************************************\
     Depois que o código for executado, o código dentro desse metodo será executado continuamente até que o botão START seja pressionado na estação do driver.init()
     \********************************************************************************************************************************************************************/
    @Override
    public void init_loop(){
        calculaTempoDeLoop();
        this.gerenciarModo(robot,gamepadEx2);
        this.gerenciarModoSuccao(robot,gamepadEx2);
        telemetry.addData(" MODO DE PONTUAÇÃO", robot.v5Mode);
        telemetry.addData(" TEMPO DE EXECUÇÃO", getRuntime());
        telemetry.addData("Distancia horizontal", robot.intakeInferior.linearHorizontalMotor.colorSensor.getDistance());
        telemetry.addData("Horizontal ta sendo visto", robot.intakeInferior.linearHorizontalMotor.sensorIsDetectingHorintontal());
        if (getRuntime() - lastUpdateTime >= updateInterval) {
            telemetry.update();
            lastUpdateTime = getRuntime();
        }
        if(!IntakeSuccao.toggle){
            telemetry.addLine("Manual");
        }
        else if(IntakeSuccao.toggle){
            telemetry.addLine("Toggle");
        }

    }


    @Override
    public void  start() {
        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.TRANSFER;
        robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL;
        robot.intakeInferior.goToInitial_goToReadyTransfer(robot.carteiro, getRuntime());
        robot.intakeInferior.linearHorizontalMotor.reset();
        if(robot.v5Mode == V5Modes.SPECIMEN){
            robot.outtakeIntakeSuperior.goToInitial(robot.carteiro, getRuntime());
        }
        if(robot.v5Mode == V5Modes.SAMPLE){
            robot.outtakeIntakeSuperior.goToReadyTransfer(robot.carteiro, 0,  getRuntime());
        }
        try {
            server = new ControlHubServer(8181); // Roda na porta 8080
        } catch (IOException e) {
            telemetry.addData("Erro", "Não foi possível iniciar o servidor!");
        }


        resetRuntime();

    }
    @Override
    public void loop() {

        calculaTempoDeLoop();
        this.gerenciarModo(robot,gamepadEx2);

        this.robotCentricDrive(robot,gamepadEx1);
        if(robot.v5Mode == V5Modes.SPECIMEN){
            this.bindsChamber(robot,gamepadEx2, robot.carteiro);
        } else if (robot.v5Mode == V5Modes.SAMPLE) {
            this.bindsSample(robot,gamepadEx2,robot.carteiro);
        }
        this.linearHorizontal(robot.carteiro,robot.intakeInferior.linearHorizontalMotor,gamepadEx2);
        this.linearVertical(robot.outtakeIntakeSuperior.linearVertical,gamepadEx2, robot.carteiro);
        this.bracoGarra(robot.outtakeIntakeSuperior.braco,gamepadEx2,robot.carteiro);
        this.IntakeSuccao(robot,robot.intakeInferior.intakeSuccao,robot.carteiro,gamepadEx2);
        this.garraSuperior(robot.outtakeIntakeSuperior.braco,robot.outtakeIntakeSuperior.garraSuperior,robot.carteiro,gamepadEx2);
        //fullAutoOuttakeChamber(robot.carteiro);
        this.hang(gamepadEx1, robot.carteiro);
        //this.verificarsample(robot,robot.carteiro, "Azul");

        this.runActions(robot.carteiro);

        telemetry.addData(" MODO DE PONTUAÇÃO", robot.v5Mode);
        telemetry.addData(" TEMPO DE EXECUÇÃO", getRuntime());


        // telemetry.addData("motor frontal direito",robot.md.rightFront.getCurrent(CurrentUnit.AMPS));
        // telemetry.addData("motor traseiro direito",robot.md.rightBack.getCurrent(CurrentUnit.AMPS));
        // telemetry.addData("motor frontal esquerdo",robot.md.leftFront.getCurrent(CurrentUnit.AMPS));
        // telemetry.addData("motor traseiro esquerdo",robot.md.leftBack.getCurrent(CurrentUnit.AMPS));





        if (getRuntime() - lastUpdateTime >= updateInterval) {
            telemetry.update();
            lastUpdateTime = getRuntime();
        }

    }

    @Override
    public void stop() {
        LinearVertical.hang = false;
        robot.carteiro.saveLogsToJson();
    }

    @NonNull
    private V5 createRobot(HardwareMap hardwareMap) {
        V5 bot = new V5(hardwareMap, telemetry);
        bot.teelop = true;
        MecanumDrive.PARAMS.maxProfileAccel = 80;
        MecanumDrive.PARAMS.minProfileAccel = -80;
        MecanumDrive.PARAMS.maxWheelVel  = 80;
        return bot;

    }
    private void robotCentricDrive(V5 robot,GamepadEx gamepad) {
        //todo: botao para andar diagonal sem alteração no heading
        robot.md.updatePoseEstimate();

        double drive = Range.clip(gamepad.getLeftY(), -1, 1);

        //if( Math.abs(drive) < 0.8 && Math.abs(drive) > 0.02  ) drive = (drive / 1.5);

        double strafe = Range.clip(-gamepad.getLeftX(), -1, 1);
        //if (gamepad1.right_stick_button) strafe = -0.6;
        //if (gamepad1.left_stick_button) strafe = 0.6;
        //if ( Math.abs(strafe) < 0.8 && Math.abs(strafe) > 0.02 ) strafe = (strafe / 2.5);


        double correction = 0;
        /*if(gamepad1.left_stick_x!=0){

            double alvo = robot.md.lazyImu.get().getRobotYawPitchRollAngles().getPitch();
            correction = alvo - robot.md.lazyImu.get().getRobotYawPitchRollAngles().getPitch();
        }*/
        double turn = Range.clip(-gamepad.getRightX()+correction,-1,1);

        if (gamepad1.left_trigger > 0) {
            strafe = -gamepad1.left_trigger * 0.5;
        }

        if (gamepad1.right_trigger > 0) {
            strafe = gamepad1.right_trigger * 0.5;
        }
        if(strafe != 0 && turn == 0){

        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            drive = powerDriveOut;
            strafe = powerStrafeOut;
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            drive = powerDriveInt;
            strafe = powerStrafeInt;
        }

        /*if(gamepad.getButton(GamepadKeys.Button.Y)){
            Actions.runBlocking(
                    new SequentialAction(
                            robot.md.actionBuilder(robot.md.pose)
                                    .setTangent(Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-5,-26),Math.toRadians(-90))
                                    //.splineToLinearHeading(new Pose2d(-5, -26, Math.toRadians(-90)), Math.toRadians(90))
                                    .build()
                    )
            );
        }*/

        /*todo rota automatica intake specime
            |
            \/

        if(gamepad.getButton(GamepadKeys.Button.Y)){
            Actions.runBlocking(
                    new SequentialAction(
                            robot.md.actionBuilder(robot.md.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(47, -50, Math.toRadians(-90)), Math.toRadians(-90))
                                    .build()
                    )
            );
        }*/
        turn = Range.clip(turn / 1.3, -0.7, 0.7);

        robot.md.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), turn));



    }
    private void testar1servo(V5 robot, GamepadEx gamepad, OrdersManager carteiro) {
    }
    private void hang(GamepadEx gamepad, OrdersManager carteiro) {
        if(gamepad.getButton(GamepadKeys.Button.X)) {
            LinearVertical.hang =false;
            robot.goReadytoHang(carteiro, getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            LinearVertical.hang =false;
            robot.goReadytoHang2(carteiro, getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.B)) {
            LinearVertical.hang =true;
            robot.goToHang(carteiro, getRuntime());
        }
    }
    private void bindsChamber(V5 robot, GamepadEx gamepad, OrdersManager carteiro) {
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            robot.intakeInferior.AbrirAlcapao(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.A)){
            LinearVertical.hang  = false;
            robot.outtakeIntakeSuperior.goToIntakeCHAMBER(carteiro, getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            LinearVertical.hang  = false;
            robot.intakeInferior.gerenciadorIntakSpecimen(carteiro,getRuntime());


        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            LinearVertical.hang  = false;
            robot.intakeInferior.goToInitial_goToReadyTransfer(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToOuttakeCHAMBER(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.B) || getRuntime() < 0.1){
            LinearVertical.hang  = false;
            robot.intakeInferior.goToInitial_goToReadyTransfer(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToIntakeCHAMBER(robot.carteiro, getRuntime());
        }

    }//todo okey
    private void bindsSample(V5 robot, GamepadEx gamepad, OrdersManager carteiro){
        if(gamepad.getButton(GamepadKeys.Button.B) || getRuntime() < 0.1){
            LinearVertical.hang  = false;
            robot.outtakeIntakeSuperior.goToReadyTransfer(carteiro, 0, getRuntime());
            robot.intakeInferior.goToInitial_goToReadyTransfer(carteiro, getRuntime());

        }
        if(gamepad.getButton(GamepadKeys.Button.A)){
            LinearVertical.hang  = false;
            robot.outtakeIntakeSuperior.CorreProTransfer(carteiro,getRuntime()
            );
        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            LinearVertical.hang  = false;
            robot.outtakeIntakeSuperior.goToReadyTransfer(carteiro, 0, getRuntime());
            robot.intakeInferior.gerenciadorIntakeSample(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            LinearVertical.hang  = false;
            robot.outtakeIntakeSuperior.goToOuttakeBASKET(carteiro,0.3,getRuntime());
        }
    }//todo errado

    private void linearVertical(LinearVertical vertical,GamepadEx gamepad, OrdersManager carteiro)  {
        vertical.PIDF();
        vertical.monitor(telemetry);
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            vertical.upSetPoint();
        } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            vertical.downSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.BACK)){

            vertical.reset();
        }

    }//todo okey
    private void bracoGarra(BracoGarraSuperior braco, GamepadEx gamepad, OrdersManager carteiro)  {
        if(gamepad.getLeftY() > 0){
            robot.outtakeIntakeSuperior.braco.upSetPoint(gamepad.getLeftY());
        }
        else if(gamepad.getLeftY() < 0){
            robot.outtakeIntakeSuperior.braco.downSetPoint(gamepad.getLeftY());
        }
        braco.monitor(telemetry);
    }
    private void linearHorizontal(OrdersManager carteiro, LinearHorizontalMotor horizontal, GamepadEx gamepad)  {
        horizontal.monitor(telemetry,"horizontal inferior");
        horizontal.PID();
        // if(gamepad.getButton(GamepadKeys.Button.A)){
        //  carteiro.addOrder(horizontal.horizontalGoTo(140),0,"horizonte",getRuntime());
        // }
        //if(gamepad.getButton(GamepadKeys.Button.B)){
        //carteiro.addOrder(horizontal.horizontalGoTo(0),0,"horizonte",getRuntime());
        // }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)){
            horizontal.downSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            horizontal.upSetPoint();
        }

        //if(gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
        // horizontal.reset();
        //}


    }
    private void garraSuperior(BracoGarraSuperior bracoGarraSuperior, GarraSuperior garra, OrdersManager carteiro, GamepadEx gamepad){
        garra.monitor(telemetry);
        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime(),bracoGarraSuperior.bracoGarraSuperiorState ),0.0,"garra superior",getRuntime());

        }
        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.gerenciadorDeRotacaoDaGarraNoTeleop(getRuntime(), robot.v5Mode),0.0,"garra superior",getRuntime());

        }
        if(gamepad.getRightY() > 0){
            garra.upSetPoint(gamepad.getRightY());
        }
        else if(gamepad.getRightY() < 0){
            garra.downSetPoint(gamepad.getRightY());
        }

        if(gamepad.getRightX() > 0.5){
            garra.upSetPointRot(gamepad.getRightX());
        }
        else if(gamepad.getRightX() < -0.5  ){
            garra.downSetPointRot(gamepad.getRightX());
        }
    }
    private void IntakeSuccao(V5 robot, IntakeSuccao intakeSuccao, OrdersManager carteiro, GamepadEx gamepad){
        intakeSuccao.monitor(telemetry);
        if(IntakeSuccao.monitor){
            robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.monitor(telemetry);
        }
        if(gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            intakeSuccao.angulacao.getController().pwmDisable();

        }

        /*if((robot.intakeInferior.linearHorizontalMotor.isBusy == true && robot.intakeInferior.linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED) || (robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.getSampleColor().equals("Não há samples por Perto") && robot.intakeInferior.underGrounSubystemStates != UnderGrounSubystemStates.INTAKE)) {
             if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0){
                 robot.intakeInferior.intakeSuccao.gerenciadorDoSugadorManual(gamepad, getRuntime());
             }
             else{
                 robot.intakeInferior.intakeSuccao.sugador.setPower(IntakeSuccao.power_Sugador / 1.5);
             }

        }*/

        //   else {
        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > IntakeSuccao.pontoAtiv || gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > IntakeSuccao.pontoAtiv){
            if(IntakeSuccao.toggle){
                robot.intakeInferior.intakeSuccao.gerenciadorDoSugador(gamepad, getRuntime());
            }

        }
        if(!IntakeSuccao.toggle){
            robot.intakeInferior.intakeSuccao.gerenciadorDoSugadorManual(gamepad, getRuntime());
        }
        // }

    }
    private void fullAutoOuttakeChamber(OrdersManager carteiro){

        if(V5.v5Mode == V5Modes.SAMPLE && robot.intakeInferior.intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER && robot.intakeInferior.linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED) {
            if (!robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.getSampleColor().equals("Não há samples por Perto")) {
                robot.outtakeIntakeSuperior.goToOuttakeBASKET(carteiro,0.3,getRuntime());
            }
        }

    }
    private void gerenciarModo(V5 robot,GamepadEx gamepadEx) {
        if(gamepadEx.getButton(GamepadKeys.Button.START)) {
            if (getRuntime() >= cooldownMode) {
                cooldownMode = getRuntime() + delayMode;

                if (this.robot.v5Mode == V5Modes.SPECIMEN) {
                    robot.v5Mode = V5Modes.SAMPLE;
                } else {
                    robot.v5Mode = V5Modes.SPECIMEN;
                }

            }
        }
    }
    private void gerenciarModoSuccao(V5 robot,GamepadEx gamepadEx) {
        if(gamepadEx.getButton(GamepadKeys.Button.BACK)) {
            if (getRuntime() >= cooldownMode) {
                cooldownMode = getRuntime() + delayMode;

                if (IntakeSuccao.toggle) {
                    IntakeSuccao.toggle = false;
                } else {
                    IntakeSuccao.toggle = true;
                }

            }
        }
    }

    private void calculaTempoDeLoop() {
        loopTime = getRuntime() - lastTime;
        telemetry.addData("Latência código", String.format("%.0f",loopTime*1000) + "ms");
        lastTime = getRuntime();
    }

    private void verificarsample(V5 robot, OrdersManager carteiro, String ladoAlianca){
        if(robot.v5Mode == V5Modes.SAMPLE && ladoAlianca.equals("Azul")){
            if(robot.intakeInferior.matchColor.isRed(robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
        //MIGUEL DESCOMENTE ISSO EU COMENTEI PRA TESTAR
        if(robot.v5Mode == V5Modes.SPECIMEN && ladoAlianca.equals("Azul")){
            if(robot.intakeInferior.matchColor.isRed(robot.intakeInferior.intakeSuccao) || robot.intakeInferior.matchColor.isYellow(robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
        if(robot.v5Mode == V5Modes.SAMPLE && ladoAlianca.equals("Red")){
            if(robot.intakeInferior.matchColor.isBlue( robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
        //MIGUEL DESCOMENTE ISSO EU COMENTEI PRA TESTAR
        if(robot.v5Mode == V5Modes.SPECIMEN && ladoAlianca.equals("Red")){
            if(robot.intakeInferior.matchColor.isBlue( robot.intakeInferior.intakeSuccao) || robot.intakeInferior.matchColor.isYellow(  robot.intakeInferior.intakeSuccao)){
                robot.intakeInferior.ejectingSampleWrong(carteiro,getRuntime(),robot.v5Mode, ladoAlianca);
            }
        }
    }
    private void runActions(OrdersManager carteiro) {
        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
    }



}