#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import typing
from ntcore import DoublePublisher, NetworkTableInstance

import wpilib

from robotcontainer import RobotContainer
from phoenix6.signal_logger import SignalLogger
from phoenix6 import units
from wpilib import DataLogManager, DriverStation, Timer
from wpinet import WebServer


class MyRobot(commands2.TimedCommandRobot):
    autonomousCommand: typing.Optional[commands2.Command] = None
    loopTimer: wpilib.Timer = None
    previousTime: units.second = 0
    loopTimePub: DoublePublisher

    def robotInit(self) -> None:
        SignalLogger.enable_auto_logging(True)
        SignalLogger.start()
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog(), True)
        WebServer.getInstance().start(5800, wpilib.getDeployDirectory())

        self.loopTimePub = (
            NetworkTableInstance.getDefault()
            .getTable("Timing")
            .getDoubleTopic("RobotPeriodicLoopRateMs")
            .publish()
        )

        self.container = RobotContainer()
        self.loopTimer = Timer()
        self.loopTimer.start()

    def robotPeriodic(self) -> None:
        currentTime = self.loopTimer.getTimestamp()
        commands2.CommandScheduler.getInstance().run()
        loopTime = currentTime - self.previousTime
        self.loopTimePub.set(loopTime * 1000)
        self.previousTime = currentTime

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
