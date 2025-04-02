#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians


class RobotContainer:
    def __init__(self) -> None:
        self._max_speed = TunerConstants.speed_at_12_volts
        self._max_angular_rate = rotationsToRadians(1.0)

        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed
                    )
                    .with_velocity_y(-self._joystick.getLeftX() * self._max_speed)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )
                )
            )
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        self._joystick.pov(0).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        self._joystick.pov(180).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )

        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self._auto_chooser.getSelected()
