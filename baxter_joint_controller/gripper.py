# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import time

from copy import deepcopy
from math import fabs
from rclpy.node import Node

from json import (
    JSONDecoder,
    JSONEncoder,
)


from baxter_core_msgs.msg import (
    EndEffectorCommand,
    EndEffectorProperties,
    EndEffectorState,
)

SDK_VERSION = '1.2.0'

class Gripper(Node):
    """
    Interface class for a gripper on the Baxter Research Robot.
    """
    def __init__(self, gripper, node, versioned=False):

        """
        Version-checking capable constructor.

        @type gripper: str
        @param gripper: robot limb <left/right> on which the gripper
                        is mounted
        @type versioned: bool
        @param versioned: True if Gripper firmware version should be checked
        on initialization. [False]

        The gripper firmware versions are checked against the version
        compatibility list in L{baxter_interface.VERSIONS_SDK2GRIPPER}.
        The compatibility list is updated in each SDK release.

        By default, this interface class will not check versions,
        but all examples using Grippers in baxter_examples pass a True
        and will check. This behavior can be overridden by setting
        L{baxter_interface.CHECK_VERSION} to False.
        """
        self.node = node
        self.name = gripper + '_gripper'
        self._cmd_sender = gripper + '_gripper_node'
        self._cmd_sequence = 0

        ns = 'robot/end_effector/' + self.name + "/"
        self.node.get_logger().info(ns + 'state')

        self.state_init = False
        self.prop_init = False
        self._state = None
        self._prop = EndEffectorProperties(id=4294967295)  # initialize w/unused value

        self._parameters = dict()

        self._cmd_pub = self.node.create_publisher(EndEffectorCommand,ns + 'command', 10)

        self._prop_pub = self.node.create_publisher(EndEffectorProperties,
                                            ns + 'rsdk/set_properties',        
                                            10
                                            )

        self._state_pub = self.node.create_publisher(EndEffectorState,
                                            ns + 'rsdk/set_state',        
                                            10
                                            )

        self._state_sub = self.node.create_subscription(EndEffectorState,
                                            ns + 'state',
                                            self._on_gripper_state,
                                            10
                                            )

        self._prop_sub = self.node.create_subscription(EndEffectorProperties,
                                            ns + 'properties',
                                            self._on_gripper_prop,
                                            10
                                            )

    def _on_gripper_state(self, state):
        self._state = deepcopy(state)
        if not self.state_init:
            self.set_parameters(defaults=True)
            self.state_init = True

    def _on_gripper_prop(self, properties):
        self._prop = deepcopy(properties)
        self.prop_init = True

    def _inc_cmd_sequence(self):
        # manage roll over with safe value (maxint)
        self._cmd_sequence = (self._cmd_sequence % 0x7FFFFFFF) + 1
        return self._cmd_sequence

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)

    def _capablity_warning(self, cmd):
        msg = ("%s %s - not capable of '%s' command" %
               (self.name, self.type(), cmd))
        self.node.get_logger().warn(msg)

    def _version_str_to_time(self, version_str, version_type):
        float_time = 0.0
        time_format = r"%Y/%m/%d %H:%M:%S"
        # strptime errors when all zeros are given,
        # so instead, return the initialized value
        # of float_time
        if version_str != '0000/0/0 0:0:00':
            try:
                float_time = time.mktime(time.strptime(version_str,
                                                       time_format))
            except ValueError:
                self.node.get_logger().error(("%s %s - The Gripper's %s "
                              "timestamp does not meet python time formating "
                              "requirements: %s does not map "
                              "to '%s'"),
                              self.name, self.type(), version_type,
                              version_str, time_format)
                sys.exit(1)
        return float_time

    # def version_check(self):
    #     """
    #     Does a safety check on the firmware build date of the electric
    #     grippers versus the version of the SDK software.

    #     @rtype: bool
    #     @return: True if gripper version is compatible with SDK version,
    #     including if in warning list or unknown.

    #     False if incompatible and in fatal fail list.
    #     """
        
    #     firmware_date_str = self.firmware_build_date()
    #     if self.type() != 'electric':
    #         self.node.get_logger().warn("%s %s (%s): Version Check not needed",
    #                       self.name, self.type(), firmware_date_str)
    #         return True
    #     if not firmware_date_str:
    #         self.node.get_logger().error("%s %s: Failed to retrieve version string during"
    #                       " Version Check.", self.name, self.type())
    #         return False
    #     firmware_time = self._version_str_to_time(
    #                      firmware_date_str,
    #                      "current firmware")
    #     warn_time = self._version_str_to_time(
    #                      settings.VERSIONS_SDK2GRIPPER[sdk_version]['warn'],
    #                      "baxter_interface settings.py firmware 'warn'")
    #     fail_time = self._version_str_to_time(
    #                      settings.VERSIONS_SDK2GRIPPER[sdk_version]['fail'],
    #                      "baxter_interface settings.py firmware 'fail'")
    #     if firmware_time > warn_time:
    #         return True
    #     elif firmware_time <= warn_time and firmware_time > fail_time:
    #         self.node.get_logger().warn("%s %s: Gripper Firmware version built on date (%s) "
    #                       "is not up-to-date for SDK Version (%s). Please use "
    #                       "the Robot's Field-Service-Menu to Upgrade your "
    #                       "Gripper Firmware.",
    #                       self.name, self.type(),
    #                       firmware_date_str, sdk_version)
    #         return True
    #     elif firmware_time <= fail_time and firmware_time > 0.0:
    #         self.node.get_logger().error("%s %s: Gripper Firmware version built on date (%s) "
    #                      "is *incompatible* with SDK Version (%s). Please use "
    #                      "the Robot's Field-Service-Menu to Upgrade your "
    #                      "Gripper Firmware.",
    #                      self.name, self.type(),
    #                      firmware_date_str, sdk_version)
    #         return False
    #     else:
    #         legacy_str = '1.1.242'
    #         if self.firmware_version()[0:len(legacy_str)] == legacy_str:
    #             # Legacy Gripper version 1.1.242 cannot be updated
    #             # This must have a Legacy Gripper build date of 0.0, 
    #             # so it passes
    #             return True
    #         else:
    #             self.node.get_logger().error("%s %s: Gripper Firmware version built on " 
    #                       "date (%s) does not fall within any known Gripper "
    #                       "Firmware Version dates for SDK (%s). Use the "
    #                       "Robot's Field-Service-Menu to Upgrade your Gripper " 
    #                       "Firmware.",
    #                       self.name, self.type(),
    #                       firmware_date_str, sdk_version)
    #             return False

    def command(self, cmd, block=False, test=lambda: True,
                 timeout=0.0, args=None):
        """
        Raw command call to directly control gripper.

        @type cmd: str
        @param cmd: string of known gripper commands
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        @type test: func
        @param test: test function for command validation
        @type timeout: float
        @param timeout: timeout in seconds for command evaluation
        @type args: dict({str:float})
        @param args: dictionary of parameter:value
        """
        ee_cmd = EndEffectorCommand()
        ee_cmd.id = self.hardware_id()
        ee_cmd.command = cmd
        ee_cmd.sender = self._cmd_sender
        ee_cmd.sequence = self._inc_cmd_sequence()
        ee_cmd.args = ''
        if args != None:
            ee_cmd.args = JSONEncoder().encode(args)
        self._cmd_pub.publish(ee_cmd)
        counter = 0
        if block:
            while not test():
                if counter == timeout*10:
                    self.node.get_logger().warn("Timeout reached")
                    return False
                counter +=1
                time.sleep(.1)
        return True

    def valid_parameters_text(self):
        """
        Text describing valid gripper parameters.

        @rtype: str
        @return: Human readable block of text describing parameters.
        Good for help text.
        """
        if self.type() == 'electric':
            return """Valid gripper parameters for the electric gripper are
            PARAMETERS:
            velocity - Velocity at which a position move will execute
            moving_force - Force threshold at which a move will stop
            holding_force - Force at which a grasp will continue holding
            dead_zone - Position deadband within move considered successful
            ALL PARAMETERS (0-100)
            """
        elif self.type() == 'suction':
            return """Valid gripper parameters for the suction gripper are
            PARAMETERS:
            vacuum_sensor_threshold - Measured suction threshold denoting grasp
            blow_off_seconds - Time in seconds to blow air on release
            ALL PARAMETERS (0-100)
            """
        else:
            return "No valid parameters for %s %s." % (self.type(), self.name)

    def valid_parameters(self):
        """
        Returns dict of available gripper parameters with default parameters.

        @rtype: dict({str:float})
        @return: valid parameters in a code-friendly dict type.
        Use this version in your programs.
        """
        valid = dict()
        if self.type() == 'electric':
            valid = dict({'velocity': 50.0,
                         'moving_force': 40.0,
                         'holding_force': 30.0,
                         'dead_zone': 5.0,
                         })
        elif self.type() == 'suction':
            valid = dict({'vacuum_sensor_threshold': 18.0,
                          'blow_off_seconds': 0.4,
                          })
        return valid

    def set_parameters(self, parameters=None, defaults=False):
        """
        Set the parameters that will describe the position command execution.

        @type parameters: dict({str:float})
        @param parameters: dictionary of parameter:value

        Percentage of maximum (0-100) for each parameter
        """
        valid_parameters = self.valid_parameters()
        if defaults:
            self._parameters = valid_parameters
        if parameters is None:
            parameters = dict()
        for key in parameters.keys():
            if key in valid_parameters.keys():
                self._parameters[key] = parameters[key]
            else:
                msg = ("Invalid parameter: %s provided. %s" %
                       (key, self.valid_parameters_text(),))
                self.node.get_logger().warn(msg)
        cmd = EndEffectorCommand.CMD_CONFIGURE
        return self.command(cmd, args=self._parameters)

    def reset_custom_properties(self, timeout=2.0):
        """
        Resets default properties for custom grippers

        @return: True if custom gripper properties reset successfully
        @rtype: bool
        """
        default_id = 131073
        default_ui_type = EndEffectorProperties.PASSIVE_GRIPPER
        default_manufacturer = 'Rethink Research Robot'
        default_product = 'SDK End Effector'
        # Create default properties message
        prop_msg = EndEffectorProperties(
                                         id=default_id,
                                         ui_type=default_ui_type,
                                         manufacturer=default_manufacturer,
                                         product=default_product,
                                         )
        for idx, attr in enumerate(prop_msg.__slots__):
            if prop_msg._slot_types[idx] == 'bool':
                setattr(prop_msg, attr, True)
        self._prop_pub.publish(prop_msg)

        return True

    def reset_custom_state(self, timeout=2.0):
        """
        Resets default state for custom grippers

        @return: True if custom gripper state reset successfully
        @rtype: bool
        """
        state_true = EndEffectorState.STATE_TRUE
        state_unknown = EndEffectorState.STATE_UNKNOWN
        # Create default state message
        state_msg = EndEffectorState()
        for idx, attr in enumerate(state_msg.__slots__):
            if 'int' in state_msg._slot_types[idx]:
                setattr(state_msg, attr, state_unknown)
        setattr(state_msg, 'enabled', state_true)
        self._state_pub.publish(state_msg)

        return True 

    def reset(self, block=True, timeout=2.0):
        """
        Resets the gripper state removing any errors.

        @type timeout: float
        @param timeout: timeout in seconds for reset success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        if self.type() != 'electric':
            return self._capablity_warning('reset')

        cmd = EndEffectorCommand.CMD_RESET
        return self.command(
                            cmd,
                            block,
                            test=lambda: (self._state.error == False and
                                          self._state.ready == True),
                            timeout=timeout,
                            )

    def _cmd_reboot(self, block=True, timeout=5.0):
        """
        Power cycle the gripper, removing calibration information.

        Basic call to the gripper reboot command. Waits for gripper to return
        ready state but does not clear errors that could occur during boot.
        Highly recommended to use the clean reboot() command instead.

        @type timeout: float
        @param timeout: timeout in seconds for reboot success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        if self.type() != 'electric':
            return self._capablity_warning('reboot')

        cmd = EndEffectorCommand.CMD_REBOOT
        success = self.command(
                      cmd,
                      block,
                      test=lambda: (self._state.enabled == True and
                                    self._state.ready == True),
                      timeout=timeout
        )
        time.sleep(1.0)  # Allow extra time for reboot to complete
        self.set_parameters(defaults=True)
        return success

    def reboot(self, timeout=5.0, delay_check=0.1):
        """
        "Clean" reboot of gripper, removes calibration and errors.

        Robust version of gripper reboot command; recommended to use this
        function for rebooting grippers.

        Calls the basic reboot gripper command to power cycle the gripper
        (_cmd_reboot()) and then checks for errors after reboot, calling
        reset to clear errors if needed.

        @type timeout: float
        @param timeout: timeouts in seconds for reboot & reset
        @type delay_check: float
        @param delay_check: seconds after reboot before error check
        """
        if self.type() != 'electric':
            return self._capablity_warning('reboot')

        self._cmd_reboot(block=True, timeout=timeout)
        time.sleep(delay_check)
        if self.error():
            if not self.reset(block=True, timeout=timeout):
                self.node.get_logger().error("Failed to reset gripper error after reboot.")
                return False
            self.set_parameters(defaults=True)
        return True

    def clear_calibration(self, block=True, timeout=2.0):
        """
        Clear calibration information from gripper.

        Allows (and requires) new gripper calibration to be run.

        @type timeout: float
        @param timeout: timeout in seconds for success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        if self.type() != 'electric':
            return self._capablity_warning('clear_calibration')
        cmd = EndEffectorCommand.CMD_CLEAR_CALIBRATION
        return self.command(
                   cmd,
                   block,
                   test=lambda: (self._state.calibrated == False and
                                 self._state.ready == True),
                   timeout=timeout
        )

    def calibrate(self, block=True, timeout=5.0):
        """
        Calibrate the gripper setting maximum and minimum travel distance.

        @type timeout: float
        @param timeout: timeout in seconds for calibration success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        @rtype: bool
        @return: Returns True if calibration succeeds.
        """
        if self.type() != 'electric':
            return self._capablity_warning('calibrate')

        # clear any previous calibration and any current errors
        if self.calibrated():
            self.clear_calibration()
        if self.error():
            self.reset(block=block)

        cmd = EndEffectorCommand.CMD_CALIBRATE
        success = self.command(
                      cmd,
                      block,
                      test=lambda: (self._state.calibrated == True and
                                    self._state.ready == True),
                      timeout=timeout
                      )
        if success:
            self.set_parameters(defaults=True)
        return success

    def stop(self, block=True, timeout=5.0):
        """
        Stop the gripper at the current position and apply holding force.

        @type timeout: float
        @param timeout: timeout in seconds for stop success
        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        if self.type() == 'custom':
            return self._capablity_warning('stop')

        if self.type() == 'electric':
            cmd = EndEffectorCommand.CMD_STOP
            stop_test = lambda: self._state.moving == False
        elif self.type() == 'suction':
            timeout = max(self._parameters['blow_off_seconds'], timeout)
            cmd = EndEffectorCommand.CMD_RELEASE
            stop_test = lambda: (not self.sucking() and not self.blowing())
        return self.command(
                            cmd,
                            block,
                            test=stop_test,
                            timeout=timeout,
                            )

    def command_position(self, position, block=False, timeout=5.0):
        """
        Command the gripper position movement.

        @type position: float
        @param position: in % 0=close 100=open

        From minimum/closed (0.0) to maximum/open (100.0)
        """
        if self.type() == 'custom':
            return self._capablity_warning('command_position')

        if self._state.calibrated != True:
            msg = "Unable to command %s position until calibrated" % self.name
            self.node.get_logger().warn(msg)
            return False

        cmd = EndEffectorCommand.CMD_GO
        arguments = {"position": self._clip(position)}
        if self.type() == 'electric':
            cmd_test = lambda: ((fabs(self._state.position - position)
                                  < self._parameters['dead_zone'])
                                 or self._state.gripping == True)
            return self.command(
                                cmd,
                                block,
                                test=cmd_test,
                                timeout=timeout,
                                args=arguments
                                )
        elif arguments['position'] < 100.0:
            return self.close(block=block, timeout=timeout)
        else:
            return self.open(block=block, timeout=timeout)

    def command_suction(self, block=False, timeout=5.0):
        """
        Command the gripper suction.

        @type timeout: float
        @param timeout: Timeout describes how long the suction will be
        applied while trying to determine a grasp (vacuum threshold exceeded)
        has been achieved.

        @type block: bool
        @param block: command is blocking or non-blocking [False]
        """
        if self.type() != 'suction':
            return self._capablity_warning('command_suction')

        cmd = EndEffectorCommand.CMD_GO
        arguments = {"grip_attempt_seconds": timeout}
        return self.command(
                            cmd,
                            block,
                            test=self.vacuum,
                            timeout=timeout,
                            args=arguments,
                            )

    def set_velocity(self, velocity):
        """
        Set the velocity at which the gripper position movement will execute.

        @type velocity: float
        @param velocity: in % 0=stop 100=max [50.0]
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_velocity')

        velocity_param = dict(velocity=self._clip(velocity))
        self.set_parameters(parameters=velocity_param, defaults=False)

    def set_moving_force(self, force):
        """
        Set the moving force threshold of the position move execution.

        When exceeded, the gripper will stop trying to achieve the commanded
        position.

        @type force: float
        @param force: in % 0=none 100=max [30.0]
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_moving_force')

        moving = dict(moving_force=self._clip(force))
        self.set_parameters(parameters=moving, defaults=False)

    def set_holding_force(self, force):
        """
        Set holding force of successful gripper grasp.

        Set the force at which the gripper will continue applying after a
        position command has completed either from successfully achieving the
        commanded position, or by exceeding the moving force threshold.

        @type force: float
        @param force: in % 0=none 100=max [30.0]
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_holding_force')

        holding = dict(holding_force=self._clip(force))
        self.set_parameters(parameters=holding, defaults=False)

    def set_dead_band(self, dead_band):
        """
        Set the gripper dead band for position moves.

        Set the gripper dead band describing the position error threshold
        where a move will be considered successful.

        @type dead_band: float
        @param dead_band: in % of full position [5.0]
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_dead_band')

        dead_band_param = dict(dead_zone=self._clip(dead_band))
        self.set_parameters(parameters=dead_band_param, defaults=False)

    def set_vacuum_threshold(self, threshold):
        """
        Set suction threshold of successful grasp.

        Set the gripper suction threshold describing the threshold at which
        the measured suction (vacuum achieved) must exceed to denote a
        successful grasp.

        @type threshold: float
        @param threshold: in % of measured vacuum range [18.0]
        """
        if self.type() != 'suction':
            return self._capablity_warning('set_vacuum_threshold')

        threshold_param = dict(vacuum_sensor_threshold=self._clip(threshold))
        self.set_parameters(parameters=threshold_param, defaults=False)

    def set_blow_off(self, blow_off):
        """
        Sets the blow_off parameter.

        This parameter will be used on a stop
        command with the suction gripper, ceasing suction and blowing air
        from the suction gripper for the seconds specified by this method.

        Note: This blow off will only be commanded after the previous suction
        command returned a successful grasp (suction threshold was exceeded)

        @type blow_off: float
        @param blow_off: Time in seconds to blow air on release [0.4]
        """
        if self.type() != 'suction':
            return self._capablity_warning('set_blow_off')

        blow_off_param = dict(blow_off_seconds=blow_off)
        self.set_parameters(parameters=blow_off_param, defaults=False)

    def open(self, block=False, timeout=5.0):
        """
        Commands maximum gripper position.

        @type block: bool
        @param block: open command is blocking or non-blocking [False]
        @type timeout: float
        @param timeout: timeout in seconds for open command success
        """
        if self.type() == 'custom':
            return self._capablity_warning('open')
        elif self.type() == 'electric':
            return self.command_position(position=100.0, block=block,
                                         timeout=timeout)
        elif self.type() == 'suction':
            return self.stop(block=block, timeout=timeout)

    def close(self, block=False, timeout=5.0):
        """
        Commands minimum gripper position.

        @type block: bool
        @param block: close command is blocking or non-blocking [False]
        @type timeout: float
        @param timeout: timeout in seconds for close command success
        """
        if self.type() == 'custom':
            return self._capablity_warning('close')
        elif self.type() == 'electric':
            return self.command_position(position=0.0, block=block,
                                         timeout=timeout)
        elif self.type() == 'suction':
            return self.command_suction(block=block, timeout=timeout)

    def parameters(self):
        """
        Returns dict of parameters describing the gripper command execution.

        @rtype: dict({str:float})
        @return: parameters describing the gripper command execution
        """
        return deepcopy(self._parameters)

    def calibrated(self):
        """
        Returns bool describing gripper calibration state.
        (0:Not Calibrated, 1:Calibrated)

        @rtype: bool
        @return: False if Not Calibrated, True if Calibrated.
        Grippers that cannot calibrate should return True
        (i.e. "Always calibrated").
        """
        if self._state is None:
            return False
        return self._state.calibrated == 1

    def ready(self):
        """
        Returns bool describing if the gripper ready, i.e. is calibrated, not
        busy (as in resetting or rebooting), and not moving.

        @rtype: bool
        @return: True if gripper is not busy
        """
        if self._state is None:
            return False

        return self._state.ready == 1

    def moving(self):
        """
        Returns bool describing if the gripper is in motion

        @rtype: bool
        @return: True if gripper is in motion
        """
        if self._state is None:
            return False

        return self._state.moving == 1

    def gripping(self):
        """
        Returns bool describing if the position move has been preempted by a
        position command exceeding the moving_force threshold denoting a grasp.

        @rtype: bool
        """
        if self._state is None:
            return False
        return self._state.gripping == 1

    def missed(self):
        """
        Returns bool describing if the position move has completed without
        exceeding the moving_force threshold denoting a grasp

        @rtype: bool
        """
        if self._state is None:
            return False
        return self._state.missed == 1

    def error(self):
        """
        Returns bool describing if the gripper is in an error state.

        Error states can be caused by over/undervoltage, over/under current,
        motor faults, etc.

        Errors can be cleared with a gripper reset. If persistent please
        contact Rethink Robotics for further debugging.

        @rtype: bool
        """
        if self._state is None:
            return False
        return self._state.error == 1

    def position(self):
        """
        Returns the current gripper position as a ratio (0-100) of the total
        gripper travel.

        @rtype: float
        """
        if self._state is None:
            return None
        return deepcopy(self._state.position)

    def force(self):
        """
        Returns the current measured gripper force as a ratio (0-100) of the
        total force applicable.

        @rtype: float
        """
        if self._state is None:
            return None
        return deepcopy(self._state.force)

    def vacuum_sensor(self):
        """
        Returns the value (0-100) of the current vacuum sensor reading as a
        percentage of the full vacuum sensor range.

        The message field contains an 8-bit integer representation of the
        vacuum sensor, this function converts that integer to the percentage of
        the full sensor range.

        @rtype: float
        """
        if self._state is None:
            return None
        if self.type() != 'suction':
            return self._capablity_warning('vacuum_sensor')
        sensor = JSONDecoder().decode(self._state.state)['vacuum sensor']
        return (sensor / 255.0) * 100.0

    def vacuum(self):
        """
        Returns bool describing if the vacuum sensor threshold has been
        exceeded during a command_suction event.

        @rtype: bool
        """
        if self._state is None:
            return None
        if self.type() != 'suction':
            return self._capablity_warning('vacuum')
        return JSONDecoder().decode(self._state.state)['vacuum']

    def blowing(self):
        """
        Returns bool describing if the gripper is currently blowing.

        @rtype: bool
        """
        if self._state is None:
            return None
        if self.type() != 'suction':
            return self._capablity_warning('blowing')
        return JSONDecoder().decode(self._state.state)['blowing']

    def sucking(self):
        """
        Returns bool describing if the gripper is currently sucking.

        @rtype: bool
        """
        if self._state is None:
            return None
        if self.type() != 'suction':
            return self._capablity_warning('sucking')
        return JSONDecoder().decode(self._state.state)['sucking']

    def has_force(self):
        """
        Returns bool describing if the gripper is capable of force control.

        @rtype: bool
        """
        if self._prop is None:
            return False
        return self._prop.controls_force == True

    def has_position(self):
        """
        Returns bool describing if the gripper is capable of position control.

        @rtype: bool
        """
        if self._state is None:
            return False
        return self._prop.controls_position == True

    def type(self):
        """
        Returns string describing the gripper type.

        Known types are 'suction', 'electric', and 'custom'. An unknown or no
        gripper attached to the research robot will be reported as 'custom'.

        @rtype: str
        """
        return {
        EndEffectorProperties.SUCTION_CUP_GRIPPER: 'suction',
        EndEffectorProperties.ELECTRIC_GRIPPER: 'electric',
        EndEffectorProperties.PASSIVE_GRIPPER: 'custom',
        }.get(self._prop.ui_type, None)

    def hardware_id(self):
        """
        Returns unique hardware id number. This is required for sending
        commands to the gripper.

        @rtype: int
        """
        if self._state is None:
            return False
        return deepcopy(self._state.id)

    def hardware_name(self):
        """
        Returns string describing the gripper hardware.

        @rtype: str
        """
        if self._prop is None:
            return False
        return deepcopy(self._prop.product)

    def firmware_build_date(self):
        """
        Returns the build date of the firmware on the current gripper.

        @rtype: str
        """
        if self._prop is None:
            return False
        return deepcopy(self._prop.firmware_date)

    def firmware_version(self):
        """
        Returns the current gripper firmware revision.

        @rtype: str
        """
        if self._prop is None:
            return False
        return deepcopy(self._prop.firmware_rev)