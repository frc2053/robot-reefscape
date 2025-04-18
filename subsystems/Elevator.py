from generated import ElevatorConstants
from phoenix6 import StatusCode, controls
from phoenix6.controls import VoltageOut
from phoenix6 import BaseStatusSignal, StatusSignal
from phoenix6.configs import Configs
from phoenix6.signals import SpnEnums
from phoenix6.hardware import TalonFX
import math
import wpilib #Maybe?
from wpilib import DataLogManager, RobotBase, RobotController, Alert
from wpimath.controller import ElevatorFeedforward, PIDController
from wpimath.trajectory import TrapezoidProfile
from commands2 import CommandPtr, Commands
from commands2.button import Trigger
import ctre

class Elevator:
    def __init__(self, display):
        self.display = display #First grap the display of the thingy
        self.configure_motors 
        self.configure_control_signals
        self.optimize_bus_singals

        self._trapSetpoint = TrapezoidProfile.State
        self._trapGoal = TrapezoidProfile.State
        self._elevatorProfile = TrapezoidProfile(TrapezoidProfile.Constraints(3.048, 6.096))
        self._elevatorVoltageSetter = VoltageOut(0)
        self._elevatorPID = PIDController
        self._followerSetter = VoltageOut(0)
        self._coastSetter = VoltageOut(0)

        self.backMotor = TalonFX(ElevatorConstants.BACK_MOTOR, "*")
        self.frontMotor = TalonFX(ElevatorConstants.FRONT_MOTOR, "*")


        self.frontMotor.set_position(0)
        self.backMotor.set_position(0)

        #finished the settings stuff

    def configure_motors(self):
        pass

    def optimize_bus_singals(self):
        BUS_UPDATE_FREQ = 250
        self.frontPositionSig = self.frontMotor.get_position()
        self.frontVelocitySig = self.frontMotor.get_velocity()
        self.frontVoltageSig = self.frontMotor.get_motor_voltage()
        self.backPositionSig = self.backMotor.get_position()
        self.backVelocitySig = self.backMotor.get_velocity()
        self.backVoltageSig = self.backMotor.get_motor_voltage()
        freqSetterStatus = BaseStatusSignal.set_update_frequency_for_all(BUS_UPDATE_FREQ, self.frontPositionSig, self.frontVelocitySig, self.frontVoltageSig, self.backPositionSig, self.backVelocitySig, self.backVoltageSig)

        DataLogManager.log(f"Set bus signal frequencies for elevator. Result was: {freqSetterStatus.name}")

        signalFrequenceAlert = Alert("Bus signal frequency issue tsk tsk sigh", Alert.AlertType.kError)


        signalFrequenceAlert.set(not freqSetterStatus.is_ok())

        optimizeFrontResult = self.frontMotor.optimize_bus_utilization()
        DataLogManager.log(f"Optimized bus signals for front elevator motor. Result was: {optimizeFrontResult.name}")

        optimizeBackResult = self.backMotor.optimize_bus_utilization()
        DataLogManager.log(f"Optimized bus signals for back elevator motor. Result was: {optimizeBackResult.name}")

        optiFrontAlert = Alert("Front elevator motor bus optimization failed womp womp", Alert.AlertType.kError)
        optiBackAlert = Alert("Back elevator motor bus optimization failed lol big sad lowkeylucky", Alert.AlertType.kError)


        optiFrontAlert.set(not optimizeFrontResult.is_ok())
        optiBackAlert.set(not optimizeBackResult.is_ok())
    
    def configure_control_signals(self):
        self._elevatorVoltageSetter.update_freq_hz = 250
        self._followerSetter.update_freq_hz = 250
        self._coastSetter.update_freq_hz = 250

    
    def periodic_update(self):
        status = BaseStatusSignal.wait_for_all(2.0/ElevatorConstants.BUS_UPDATE_FREQ, self.frontPositionSig, self.frontVelocitySig, self.frontVoltageSig, self.backPositionSig, self.backVelocitySig, self.backVoltageSig)

        if not (status.IsOK()):
            DataLogManager.Log(f"Error updating elevator positions! Error was: {status.GetName()}")
        
        self.backMotor.SetControl(self.followerSetter)

        self.currentHeight = self.GetHeight()

        #continued on line 59
        next = self._elevatorProfile.Calculate(0.02, self._trapSetpoint, self._trapGoal) #I have no idea what a trapezoid profile is so this is it for now lol
        #line 61? idk

        currentKg = 120000
        kP = 1
        kI = 2
        kD = 3 #originally currentGains.kP.value() 
        kS = 12
        vA = 10
        kA = 0 #?
        isCharacterizing = False

        NUM_OF_STAGES = 2
        PULLEY_DIAM = 1

        ff = ElevatorFeedforward(kS, currentKg, vA, kA) #grabbing instance? idk if needed cause idk why the str import not working :/
        def GetElevatorVel():
            avgVel = ConvertEncVelToHeightVel(self.frontVelocitySig.get_value() + self.backVelocitySig.get_value()) / 2.0 * NUM_OF_STAGES
            return avgVel

        def ConvertEncVelToHeightVel(radialVel):
            ret = (radialVel / math.radians(180)) * (PULLEY_DIAM / 2.0)
            return ret
        
        ffToSend = 0
        ffToSend = ff.calculate(GetElevatorVel(), next.velocity)

        elevatorPid = wpilib.PIDController(kP,kI, kD)
        
        pidOutput = elevatorPid.calculate(self.currentHeight.value(), next.position.value())


        if not (isCharacterizing):
            self.frontMotor.set_control(self._elevatorVoltageSetter.with_output(ffToSend + pidOutput).with_enable_foc(True))
        else:
            pass

        height_tolerance = 0.5 #inches

        isAtGoalHeight = False
        isAtGoalHeight = abs(self.trapGoal.position - self.currentHeight) < height_tolerance
        
        def set_elevator_height(new_height):
            pass

        self.display(set_elevator_height(self.currentHeight))

        UpdateNTEntries()

        trapSetpoint = next
    
    def UpdateNTEntries():
        pass






