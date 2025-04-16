from constants import ElevatorConstants as consts
from phoenix6 import StatusCode, controls
from phoenix6.controls import VoltageOut
from phoenix6 import BaseStatusSignal
from phoenix6.configs import Configs
from phoenix6.signals import SpnEnums
import math
import wpilib #Maybe?
from wpilib import DataLogManager, RobotBase, RobotController
from wpimath.controller import ElevatorFeedforward, PIDController
from wpimath.trajectory import ExponentialProfile, TrapezoidProfile
from commands2 import CommandPtr, Commands
from commands2.button import Trigger
from str import GainTypes

class Elevator:
    def __init__(self,display):
        self.display = display #First grap the display of the thingy
        self.ConfigureMotors 
        self.ConfigureControlSignals
        self.OptimiveBusSignals
        self._trapSetpoint = TrapezoidProfile.State
        self._trapGoal = TrapezoidProfile.State
        self._elevatorProfile = TrapezoidProfile(TrapezoidProfile.Constraints(3.048, 6.096))
        self._elevatorVoltageSetter = VoltageOut(0)
        self._elevatorPID = PIDController

        self.frontMotor.SetPosition(0) #might have to do 0* length.turn???  idk
        self.backMotor.SetPosition(0)

        #finished the settings stuff

    
    def periodic_update(self):
        status = BaseStatusSignal.wait_for_all(self.FrontvoltagesSig, self.backPositionSig, self.backVelocitySig, self.backVoltageSig)

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
            avgVel = ConvertEncVelToHeightVel(self.frontVelocitySig.GetValue() + self.backVelocitySig.GetValue()) / 2.0 * NUM_OF_STAGES
            return avgVel

        def ConvertEncVelToHeightVel(radialVel):
            ret = (radialVel / math.radians(180)) * (PULLEY_DIAM / 2.0)
            return ret
        
        ffToSend = 0
        ffToSend = ff.Calculate(GetElevatorVel(), next.velocity)

        elevatorPid = wpilib.PIDController(kP,kI, kD)
        
        pidOutput = elevatorPid.Calculate(self.currentHeight.value(), next.position.value())


        if not (isCharacterizing):
            self.frontMotor.SetControl(self._elevatorVoltageSetter.with_output(ffToSend + pidOutput).with_enable_foc(True))
        else:
            pass

        height_tolerance = 0.5 #inches

        isAtGoalHeight = False
        isAtGoalHeigh = abs(self.trapGoal.position - self.currentHeight) < height_tolerance
        
        def set_elevator_height(new_height):
            pass

        self.display(set_elevator_height(self.currentHeight))

        UpdateNTEntries()

        trapSetpoint = next
    
    def UpdateNTEntries():
        pass






