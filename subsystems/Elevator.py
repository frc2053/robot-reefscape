from constants import ElevatorConstants as consts
from phoenix import StatusCodes
from phoenix6 import StatusSignal
from phoenix6.configs import Configs
from phoenix6.signals import SpnEnums
from frc import DataLogManager, RobotBase, RobotController
from frc.controller import ElevatorFeedforward
from frc.trajectory import ExponentialProfile, TrapezoidProfile
from frc2.command import CommandPtr, Commands
from frc2.command.button import Trigger
from str import GainTypes, Units
from units import acceleration, angle, angular_velocity, current, length, time, velocity, voltage
from units import math as units_math


class Elevator:
    def __init__(self,display):
        self.display = display
        self.ConfigureMotors
        self.ConfigureControlSignals
        self.OptimiveBusSignals

        self.frontMotor.SetPosition(0) #might have to do 0* length.turn
        self.backMotor.SetPosition(0)

    
    def __init__(Periodic):
        status = ctre.StatusCode(BaseStatusSignal.WaitForAll)( #extremely questionable - needs to develop library first 
            2.0/consts.Elevator.BUS_UPDATE_FREQ, self.frontPositionSig, self.frontVelocitySig,
            self.FrontvoltagesSig, self.backPositionSig, self.backVelocitySig, self.backVoltageSig)
        

        if not (status.IsOK()):
            DataLogManager.Log(f"Error updating elevator positions! Error was: {status.GetName()}")
        
        self.backMotor.SetControl(self.followerSetter)

        self.currentHeight = self.GetHeight()

        #continued on line 59

        next = self.trapProf.Calculate(20 * time.ms, self.trapSetpoint, self.trapGoal)
        #line 61? idk

