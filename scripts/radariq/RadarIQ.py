# This file is part of RadarIQ SDK
# (C) 2019 RadarIQ <support@radariq.io>
#
# SPDX-License-Identifier:    MIT

from __future__ import division
import logging
import time
from radariq.compatability import pack, unpack, as_hex, int_to_bytes
from radariq.TSerial import TSerial, MODE_BSL
import radariq.units_converter as units
from radariq.port_manager import find_com_port
import numpy as np

# Capture modes
MODE_POINT_CLOUD = 0
MODE_OBJECT_TRACKING = 1

# Moving object options
MOVING_BOTH = 0
MOVING_OBJECTS_ONLY = 1

# Reset Codes
RESET_REBOOT = 0
RESET_FACTORY_SETTINGS = 1

# Point Densities
DENSITY_NORMAL = 0
DENSITY_DENSE = 1
DENSITY_VERY_DENSE = 2

# Python output formats
OUTPUT_LIST = 0
OUTPUT_NUMPY = 1

log = logging.getLogger('RadarIQ')
# @todo message type handeling
class RadarIQ:
    """
    API Wrapper for RadarIQ
    """

    def __init__(self, port=None, *args, **kwargs):
        """
        Initializes a RadarIQ object.

        :param port: COM port the RadarIQ module is attached to.
                     If not supplied, then the module is searched for automatically
        :type port: str or None
        """
        if port is None:
            p = find_com_port()
            port = p.device

        self.connection = TSerial(port=port, baudrate=115200, mode=MODE_BSL, *args, **kwargs)
        self.distance_units = "m"
        self.speed_units = "m/s"
        self.is_capturing = False
        self.capture_max = 0
        self.capture_count = 0
        self.timeout = 5
        self.capture_mode = MODE_POINT_CLOUD
        self.output_format = OUTPUT_NUMPY # todo make this configurable

    def __del__(self):
        print("destructor called")
        try:
            self.stop()
            self.connection.stop()
            self.connection.close()
        except Exception:
            pass

    def _send(self, msg):
        """
        Send a message on to the serial bus
        :param msg: message to send
        // @todo :type msg: bytes
        """
        print("Sending: "+ as_hex(msg))
        self.connection.flush_all()
        self.connection.send_bsl_packet(msg)

    def _read(self):
        """
        Read a message from the queue.

        :return: message
        :rtype: bytes
        """
        start_time = time.time()
        while time.time() < (start_time + self.timeout):
            msg = self.connection.read_from_queue()

            if msg is not None:
                print("Receiving:", as_hex(msg))
                if msg[0:1] == int_to_bytes(0x00):  # Message packet. Send to log instead of processing normally
                    self.process_message(msg)
                else:
                    return msg
        raise Exception("Timeout while reading from the RadarIQ module")

    def set_units(self, distance_units, speed_units):
        """
        Sets the units this instance will use.

        These units are used in with settings and the data returned.
        By default SI units (m and m/s) are used. It is a good idea to set the units before any other commands are sent
        if using non-default units.

        :param distance_units: The distance units to use:  "mm", "cm", "m", "km", "in", "ft", "mi"
        :type distance_units: str
        :param speed_units: The speed units to use: "mm/s", "cm/s", "m/s", "km/h", "in/s", "ft/s", "mi/h"
        :type speed_units: str
        """
        try:
            # Performing a units conversion will throw an exception if the units are not valid
            if distance_units is not None:
                units.convert_distance_to_si(distance_units, 1)
                self.distance_units = distance_units

            if speed_units is not None:
                units.convert_speed_to_si(speed_units, 1)
                self.speed_units = speed_units
        except ValueError as err:
            raise ValueError(err)

    def process_message(self, msg):
        """
        Process a message onto the python log
        """
        try:
            print('trying')
            res = unpack("<BBBB200s", msg)
            if res[0] == 0x00 and res[1] == 0x01:
                message_type = res[2]
                message_code = res[3]
                message = res[4]
                if message_type in [0, 1]:
                    log.debug('{} {}'.format(message_code, message))
                elif message_type in [2, 5]:
                    log.info('{} {}'.format(message_code, message))
                elif message_type in 3:
                    log.warning('{} {}'.format(message_code, message))
                elif message_type in 4:
                    log.error('{} {}'.format(message_code, message))
        except Exception:
            raise Exception("Failed to process message from the RadarIQ module")

    def get_version(self):
        """
        Gets the version of the hardware and firmware.

        :return: The module version (firmware and hardware)
        :rtype: str
        """
        try:
            self._send(pack("<BB", 0x01, 0x00))
            res = unpack("<BBBBHBBH", self._read())
            if res[0] == 0x01 and res[1] == 0x01:
                return "Firmware: {}.{}.{} Hardware: {}.{}.{}".format(res[2], res[3], res[4], res[5], res[6], res[7])
            else:
                raise Exception("Invalid response")

        except Exception:
            raise Exception("Failed to get version")

    def get_serial_number(self):
        """
        Gets the serial of the module.

        :return: The modules serial number
        :rtype: str
        """
        try:
            self._send(pack("<BB", 0x02, 0x00))
            res = unpack("<BBQQ", self._read())
            if res[0] == 0x02 and res[1] == 0x01:
                return "{}-{}".format(res[2], res[3])
            else:
                raise Exception("Invalid response")

        except Exception:
            raise Exception("Failed to get serial")

    def reset(self, code):
        """
        Resets the module.

        :param code: The command code @todo list these out
        :type code: int
        """
        if not 0 <= code <= 1:
            raise ValueError("Invalid reset code")

        try:
            self._send(pack("<BBB", 0x03, 0x02, code))
            res = unpack("<BB", self._read())
            if res[0] == 0x03 and res[1] == 0x01:
                return True
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to reset module")

    def get_frame_rate(self):
        """
        Gets the frequency with which to capture frames of data(frames/second) from the module.

        :return: The frame rate as it is set in the module
        :rtype: int
        """
        try:
            self._send(pack("<BB", 0x04, 0x00))
            res = unpack("<BBB", self._read())
            if res[0] == 0x04 and res[1] == 0x01:
                return res[2]
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get frame rate")

    def set_frame_rate(self, frame_rate):
        """
        Sets the frequency with which to capture frames of data.

        :param frame_rate: The frame rate frames/second)
        :type frame_rate: int
        """

        if not isinstance(frame_rate, int):
            raise ValueError("Frame rate must be an integer")

        if not 0 <= frame_rate <= 30:
            raise ValueError("Frame rate must be between 0 and 30 fps")

        try:
            self._send(pack("<BBB", 0x04, 0x02, frame_rate))
            res = unpack("<BBB", self._read())
            if res[0] == 0x04 and res[1] == 0x01:
                if res[2] == frame_rate:
                    return True
                else:
                    raise Exception("Frame rate did not set correctly")
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to set frame rate")

    def get_mode(self):
        """
        Gets the capture mode from the module.

        :return: The capture mode which will be one of MODE_POINT_CLOUD or MODE_OBJECT_TRACKING
        :rtype: int
        """
        try:
            self._send(pack("<BB", 0x05, 0x00))
            res = unpack("<BBB", self._read())
            if res[0] == 0x05 and res[1] == 0x01:
                return res[2]
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get mode")

    def set_mode(self, mode):
        """
        Sets the capture mode for the module.

        :param mode: One of MODE_POINT_CLOUD or MODE_OBJECT_TRACKING
        :type mode: int
        :return: None
        """
        if not 0 <= mode <= 1:
            raise ValueError("Invalid mode")

        try:
            self._send(pack("<BBB", 0x05, 0x02, mode))
            res = unpack("<BBB", self._read())
            if res[0] == 0x05 and res[1] == 0x01:
                if res[2] == mode:
                    return True
                else:
                    raise Exception("Mode did not set correctly")
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to set mode")

    def get_distance_filter(self):
        """
        Gets the distance filter applied to the readings.

        :return: The distance filter
                 eg. ``{"minimum": minimum, "maximum": maximum}``
        :rtype: dict
        """
        try:
            self._send(pack("<BB", 0x06, 0x00))
            res = unpack("<BBHH", self._read())
            if res[0] == 0x06 and res[1] == 0x01:
                minimum = units.convert_distance_from_si(self.distance_units, res[2] / 1000)
                maximum = units.convert_distance_from_si(self.distance_units, res[3] / 1000)
                return {"minimum": minimum, "maximum": maximum}
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get distance filter")

    def set_distance_filter(self, minimum, maximum):
        """
        Sets the distance filter applied to the readings.

        :param minimum: The minimum distance (in units as specified by set_units()
        :type minimum: number
        :param maximum: The maximum distance (in units as specified by set_units()
        :type maximum: number
        """
        minimum = int(units.convert_distance_to_si(self.distance_units, minimum) * 1000)
        maximum = int(units.convert_distance_to_si(self.distance_units, maximum) * 1000)

        if not (isinstance(minimum, int) and 0 <= minimum <= 10000):
            raise ValueError("Distance filter minimum must be a number between 0 and 10000mm")

        if not (isinstance(maximum, int) and 0 <= maximum <= 10000):
            raise ValueError("Distance filter maximum must be a number between 0 and 10000mm")

        if maximum < minimum:
            raise ValueError("Distance filter maximum must be greater than the minimum")

        try:

            self._send(pack("<BBHH", 0x06, 0x02, minimum, maximum))
            res = unpack("<BBHH", self._read())
            if res[0] == 0x06 and res[1] == 0x01:
                if res[2] == minimum and res[3] == maximum:
                    return True
                else:
                    raise Exception("Distance filter did not set correctly")
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to set distance filter")

    def get_angle_filter(self):
        """
        Gets the angle filter applied to the readings.

        :return: The angle filter. eg. ``{"minimum": minimum, "maximum": maximum}``
        :rtype: dict
        """
        try:
            self._send(pack("<BB", 0x07, 0x00))
            res = unpack("<BBbb", self._read())
            if res[0] == 0x07 and res[1] == 0x01:
                return {"minimum": res[2], "maximum": res[3]}
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get angle filter")

    def set_angle_filter(self, minimum, maximum):
        """
        Sets the angle filter to apply to the readings.

        :param minimum: The minimum angle (-60 to +60)
        :type minimum: int
        :param maximum: The maximum angle (-60 to +60)
        :type maximum: int
        """
        if not (isinstance(minimum, int) and -55 <= minimum <= 55):
            raise ValueError("Angle filter minimum must be an integer between -55 and +55")

        if not (isinstance(maximum, int) and -55 <= maximum <= 55):
            raise ValueError("Angle filter maximum must be an integer between -55 and +55")

        if maximum < minimum:
            raise ValueError("Angle filter maximum must be greater than the minimum")

        try:
            self._send(pack("<BBbb", 0x07, 0x02, minimum, maximum))
            res = unpack("<BBbb", self._read())
            if res[0] == 0x07 and res[1] == 0x01:
                return True
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get angle filter")

    def get_moving_filter(self):
        """
        Gets the moving objects filter applied to the readings.

        :return: moving filter
        :rtype: str
        """
        try:
            self._send(pack("<BB", 0x08, 0x00))
            res = unpack("<BBB", self._read())
            if res[0] == 0x08 and res[1] == 0x01:
                return res[2]
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get moving filter")

    def set_moving_filter(self, moving):
        """
        Sets the moving filter to apply to the readings.

        :param moving: One of MOVING_BOTH, MOVING_OBJECTS_ONLY
        :type moving: int
        """

        if not (isinstance(moving, int) and 0 <= moving <= 1):
            raise ValueError("Moving filter value is invalid")
        try:
            self._send(pack("<BBB", 0x08, 0x02, moving))
            res = unpack("<BBB", self._read())
            if res[0] == 0x08 and res[1] == 0x01:
                if res[2] == moving:
                    return True
                else:
                    raise Exception("Moving filter did not set correctly")
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to set moving filter")

    def save(self):
        """
        Saves the settings to the module.
        """
        try:
            self._send(pack("<BB", 0x09, 0x02))
            res = unpack("<BB", self._read())
            if res[0] == 0x09 and res[1] == 0x01:
                return True
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to save settings")

    def get_point_density(self):
        """
        Gets the point density setting.

        :return: Point density
        :rtype: int
        """
        try:
            self._send(pack("<BB", 0x10, 0x00))
            res = unpack("<BBB", self._read())
            if res[0] == 0x10 and res[1] == 0x01:
                return res[2]
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get point density setting")

    def set_point_density(self, density):
        """
        Sets the point density setting.

        :param density: The point density to set
        :type density: int
        """
        if not 0 <= density <= 2:
            raise ValueError("Invalid point density setting")

        try:
            self._send(pack("<BBB", 0x10, 0x02, density))
            res = unpack("<BBB", self._read())
            if res[0] == 0x10 and res[1] == 0x01:
                if res[2] == density:
                    return True
                else:
                    raise Exception("Point density did not set correctly")
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to set the point density")

    def get_certainty(self):
        """
        Gets the point certainty setting.

        :return: Certainty setting
        :rtype: int
        """
        try:
            self._send(pack("<BB", 0x11, 0x00))
            res = unpack("<BBB", self._read())
            if res[0] == 0x11 and res[1] == 0x01:
                return res[2]
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get certainty setting")

    def set_certainty(self, certainty):
        """
        Sets the the certainty setting to apply to the readings.

        :param certainty: The certainty setting to set
        :type certainty: int
        """
        if not (isinstance(certainty, int) and 0 <= certainty <= 9):
            raise ValueError("Certainty must be an integer between 0 and 9")

        try:
            self._send(pack("<BBB", 0x11, 0x02, certainty))
            res = unpack("<BBB", self._read())
            if res[0] == 0x11 and res[1] == 0x01:
                if res[2] == certainty:
                    return True
                else:
                    raise Exception("Certainty setting did not set correctly")
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to set the certainty setting")

    def start(self, samples=0, clear_buffer=True):
        """
        Start to capture data into the queue. To fetch data use get_data() and to stop capture call stop_capture().

        When capturing data in a non-continuous mode. The data capture will automatically stop once the number of
        samples has been received.

        :param samples: The number of samples to capture (0 = continuous)
        :type samples: int
        :param clear_buffer: When set any data currently on the buffer will be cleared before beginning capture
        :type clear_buffer: bool
        """
        try:
            if clear_buffer is True:
                self.connection.emtpy_queue()
            self._send(pack("<BBB", 0x64, 0x00, samples))
            self.is_capturing = True
            self.capture_max = samples
            self.capture_count = 0
        except Exception:
            raise Exception("Failed to start data capture")

    def stop(self):
        """
        Stops capturing of data.
        """
        try:
            self._send(pack("<BB", 0x65, 0x00))
            self.is_capturing = False
        except Exception:
            raise Exception("Failed to stop data capture")

    def get_data(self, timeout=10):
        """
        Fetches data from the queue.

        :param timeout: Maximum of seconds to wait for data.
        :return: A frame of data
        :rtype: ndarray or None (on timeout)
        """

        if timeout <= 0:
            raise Exception("Timeout must be greater than 0")

        last_data_time = time.time()
        rx_frame = []
        while self.is_capturing is True and time.time() < (last_data_time + timeout):
            try:
                subframe = self.connection.read_from_queue()
                if subframe is not None:
                    header = subframe[:4]
                    (command, variant, subframe_type, point_count) = unpack("<BBBB", header)
                    if command == 0x66 and variant == 0x01:  # is a point cloud packet
                        unpacking = "<" + "hhhB" * point_count
                        unpacked = unpack(unpacking, subframe[4:])
                        idx = 0
                        for cnt in range(point_count):
                            # SI units are needed so convert mm to m
                            rx_frame.append([units.convert_distance_from_si(self.distance_units, unpacked[idx] / 1000),
                                             units.convert_distance_from_si(self.distance_units, unpacked[idx + 1] / 1000),
                                             units.convert_distance_from_si(self.distance_units, unpacked[idx + 2] / 1000),
                                             unpacked[idx + 3]])
                            idx += 4

                        if subframe_type == 0x02:  # End of frame
                            self.capture_count += 1
                            if self.output_format == OUTPUT_LIST: # @todo put tests around this
                                yield rx_frame
                            elif self.output_format == OUTPUT_NUMPY:
                                yield self._convert_to_numpy(rx_frame)

                            rx_frame = []  # clear the buffer now the frame has been sent

                        if 0 < self.capture_max == self.capture_count:
                            break

                        last_data_time = time.time()
                    else:
                        pass
            except ValueError:
                pass

    def _convert_to_numpy(self, frame):
        """
        Convert the whole frame from a Python list to a numpy array
        :param frame: Frame to convert
        :return: Data as a numpy array [[x0, y0, z0,intensity0], ...]
        :rtype: ndarray
        """

        cnt = len(frame)
        data = np.zeros((cnt, 4))
        for idx, point in enumerate(frame):
            data[idx][0] = point[0]
            data[idx][1] = point[1]
            data[idx][2] = point[2]
            data[idx][3] = point[3]
        return data
