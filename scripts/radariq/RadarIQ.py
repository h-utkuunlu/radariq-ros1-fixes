# This file is part of RadarIQ SDK
# (C) 2019 RadarIQ <support@radariq.io>
#
# SPDX-License-Identifier:    MIT

from __future__ import division
import logging
import time
import threading
from radariq.compatability import pack, unpack, as_hex, int_to_bytes, queue
from radariq.TSerial import TSerial, CONNECTION_DISCONNECTED
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
    API Wrapper for the RadarIQ-M1 sensor

    :param port: COM port the RadarIQ sensor is attached to.
                 If not supplied, then the sensor is searched for automatically.
                 eg. /dev/ttyUSBx on Linux or COMx on Windows.
    :type port: str
    :param output_format: The format for the outputted points. See `constants`_ for options.
                          defaults to OUTPUT_LIST.
    :type output_format: int
    :param connection_status_callback: (Optional) Function to call when the connection to the RadarIQ sensor changes.
    :type connection_status_callback: def
    :param queue_length: The maximum length of the data queue.
                         Set to 0 to buffer all data until it has been processed or set to a lower number to discard
                         data if the queue is too long.
                         This is useful to prevent the data getting out of sync with reality if the consuming
                         application is not consuming the data fast enough.
    :type queue_length: int
    """

    def __init__(self, port=None, output_format=OUTPUT_LIST, connection_status_callback=None, queue_length=2, *args,
                 **kwargs):
        """
        Initializes a RadarIQ object.

        """
        if port is None:
            p = find_com_port()
            port = p.device

        self.application_connection_status_callback = connection_status_callback
        self.connection = TSerial(port=port, baudrate=115200,
                                  connection_status_callback=self._connection_status_callback, *args, **kwargs)
        self.data_queue = queue.Queue(queue_length)
        self.distance_units = "m"
        self.speed_units = "m/s"
        self.acceleration_units = 'm/s^2'
        self.mirror = False
        self.is_capturing = False
        self.capture_max = 0
        self.capture_count = 0
        self.timeout = 5
        self.capture_mode = MODE_POINT_CLOUD
        self.output_format = output_format
        self.statistics = None

        self.clean_start()

    def __del__(self):
        self.close()

    def clean_start(self):
        self.stop()
        time.sleep(0.5)  # wait for the sensor to stop (if it is running)
        self.connection.emtpy_queue()

    def close(self):
        """
        Stop the RadarIQ sensor and close serial connection.
        This should be called as part of a programs shutdown routine.
        """
        try:
            self.stop()
            time.sleep(0.5)  # wait for the sensor to stop before closing the serial connection
            self.connection.stop()
            self.connection.close()
        except Exception:
            pass

    def _send(self, msg):
        """
        Send a message on to the serial bus.

        :param msg: message to send
        :type msg: bytes
        """
        print("Sending: " + as_hex(msg))
        self.connection.flush_all()
        self.connection.send_packet(msg)

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
                    self._process_message(msg)
                else:
                    return msg
        raise Exception("Timeout while reading from the RadarIQ sensor")

    def get_mirror(self):
        """
        Gets the mirror setting.

        True = data is mirrored in the X-dimension.

        False = no mirroring.

        :return: The mirror status.
        :rtype: bool
        """
        return self.mirror

    def set_mirror(self, mirror=False):
        """
        Enable the coordinates in the X-dimension to be mirrored.

        :param mirror: set to True to mirror the radar data in the X direction.
        :type mirror: bool
        """
        self.mirror = mirror

    def set_units(self, distance_units=None, speed_units=None, acceleration_units=None):
        """
        Sets the units this instance will use.

        These units are used in with settings and the data returned.
        By default SI units (m and m/s) are used. It is a good idea to set the units before any other commands are sent
        if using non-default units.

        :param distance_units: The distance units to use:  "mm", "cm", "m", "km", "in", "ft", "mi"
        :type distance_units: str
        :param speed_units: The speed units to use: "mm/s", "cm/s", "m/s", "km/h", "in/s", "ft/s", "mi/h"
        :type speed_units: str
        :param acceleration_units: The speed units to use: "mm/s^2", "m/s^2", "in/s^2", "ft/s^2"
        :type acceleration_units: str
        """
        try:
            # Performing a units conversion will throw an exception if the units are not valid
            if distance_units is not None:
                units.convert_distance_to_si(distance_units, 1)
                self.distance_units = distance_units

            if speed_units is not None:
                units.convert_speed_to_si(speed_units, 1)
                self.speed_units = speed_units

            if acceleration_units is not None:
                units.convert_acceleration_to_si(acceleration_units, 1)
                self.acceleration_units = acceleration_units
        except ValueError as err:
            raise ValueError(err)

    def _process_message(self, msg):
        """
        Process a message onto the python log.
        """
        try:
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
            raise Exception("Failed to process message from the RadarIQ sensor")

    def get_version(self):
        """
        Gets the version of the hardware and firmware.

        :return: The sensor version (firmware and hardware)
        :rtype: dict
        """
        try:
            self._send(pack("<BB", 0x01, 0x00))
            res = unpack("<BBBBHBBH", self._read())
            if res[0] == 0x01 and res[1] == 0x01:
                return {"firmware": list(res[2:5]), "hardware": list(res[5:8])}
            else:
                raise Exception("Invalid response")

        except Exception:
            raise Exception("Failed to get version")

    def get_radar_application_versions(self):
        """
        Gets the version of the radar applications.

        :return: The radar application versions
        :rtype: dict
        """
        try:
            self._send(pack("<BB", 0x14, 0x00))
            res = unpack("<BBH20sBBH20sBBH20sBBH", self._read())
            if res[0] == 0x14 and res[1] == 0x01:
                return {'controller': res[2:5],
                        'application_1': res[5:9],
                        'application_2': res[9:13],
                        'application_3': res[13:17]
                        }
            else:
                raise Exception("Invalid response")

        except Exception:
            raise Exception("Failed to get radar versions")

    def get_serial_number(self):
        """
        Gets the serial of the sensor.

        :return: The sensors serial number
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
        Resets the sensor.

        :param code: The reset code. See `Reset codes`_
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
            raise Exception("Failed to reset sensor")

    def get_frame_rate(self):
        """
        Gets the frequency with which to capture frames of data (frames/second) from the sensor.

        :return: The frame rate as it is set in the sensor
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
        Gets the capture mode from the sensor.

        :return: The capture mode. See `Capture Modes`_
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
        Sets the capture mode for the sensor.

        :param mode: See `Capture Modes`_
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
        The units of this filter are those set by :meth:`set_units`


        :return: The distance filter

            .. code-block:: python

                 {"minimum": minimum,
                  "maximum": maximum}

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

        :param minimum: The minimum distance (in units as specified by :meth:`set_units`)
        :type minimum: number
        :param maximum: The maximum distance (in units as specified by :meth:`set_units`)
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
        Gets the angle filter applied to the readings (in degrees).

        0 degrees is center, negative angles are to the left of the sensor, positive angles are to the right.

        :return: The angle filter.
         .. code-block:: python

            {"minimum": minimum,
             "maximum": maximum}

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
        Sets the angle filter to apply to the readings (in degrees).

        0 degrees is center, negative angles are to the left of the sensor, positive angles are to the right.


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

        :return: moving filter. See `Moving filter`_
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
        Saves the settings to the sensor.
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

        :return: Point density. See `Point Density Options`_
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

        :param density: The point density to set. See `Point Density Options`_
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
        Get the point certainty setting.

        :return: Certainty setting. See `Certainty values`_.
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

        :param certainty: The certainty setting to set. See `Certainty values`_.
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

    def get_height_filter(self):
        """
        Gets the height filter applied to the readings.
        The units of this filter are those set by :meth:`set_units`

        :return: The height filter

            .. code-block:: python

                {"minimum": minimum,
                "maximum": maximum}

        :rtype: dict
        """
        try:
            self._send(pack("<BB", 0x12, 0x00))
            res = unpack("<BBhh", self._read())
            if res[0] == 0x12 and res[1] == 0x01:
                minimum = units.convert_distance_from_si(self.distance_units, res[2] / 1000)
                maximum = units.convert_distance_from_si(self.distance_units, res[3] / 1000)
                return {"minimum": minimum, "maximum": maximum}
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to get height filter")

    def set_height_filter(self, minimum, maximum):
        """
        Sets the height filter applied to the readings.

        :param minimum: The minimum height (in units as specified by set_units()
        :type minimum: number
        :param maximum: The maximum height (in units as specified by set_units()
        :type maximum: number
        """
        minimum = int(units.convert_distance_to_si(self.distance_units, minimum) * 1000)
        maximum = int(units.convert_distance_to_si(self.distance_units, maximum) * 1000)

        if not (isinstance(minimum, int)):
            raise ValueError("Height filter minimum must be a number")

        if not (isinstance(maximum, int)):
            raise ValueError("Height filter maximum must be a number")

        if maximum < minimum:
            raise ValueError("Height filter maximum must be greater than the minimum")

        try:

            self._send(pack("<BBhh", 0x12, 0x02, minimum, maximum))
            res = unpack("<BBhh", self._read())
            if res[0] == 0x12 and res[1] == 0x01:
                if res[2] == minimum and res[3] == maximum:
                    return True
                else:
                    raise Exception("Height filter did not set correctly")
            else:
                raise Exception("Invalid response")
        except Exception:
            raise Exception("Failed to set height filter")

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
                self.data_queue.empty()
            self._send(pack("<BBB", 0x64, 0x00, samples))
            self.is_capturing = True
            t = threading.Thread(target=self._get_data_thread)
            t.start()
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
        except Exception:
            raise Exception("Failed to stop data capture")
        self.is_capturing = False

    def get_data(self):
        """
        Fetches data from the RadarIQ sensor as a generator.

        :meth:`get_data` is a generator, which means it should be used like an iterable:

        .. code-block:: python

            for frame in get_data():
                print(frame)
            print ("The generator stops when there is no more data")


        The return type from :meth:`get_data` depends on the output_type set during the initialisation of this class
        and the mode the sensor is in (point cloud or object tracking).

        **Point Cloud**

        *As a list OR ndarray:*

        .. code-block:: python

            [[x, y, z, speed, intensity]..]

        **Object Tracking**

        *As a list:*

        .. code-block:: python

            [{'tracking_id': tracking_id,
              'x_pos': x_pos,
              'y_pos': y_pos,
              'z_pos': z_pos,
              'x_vel': x_vel,
              'y_vel': y_vel,
              'z_vel': z_vel,
              'x_acc': x_acc,
              'y_acc': y_acc,
              'z_acc': z_acc,
            }]

        *As an ndarray:*

        .. code-block:: python

            [[tracking_id,x_pos, y_pos, z_pos, x_vel, y_vel, z_vel, x_acc,y_acc, z_acc]...]

        :return: A frame of data
        :rtype: a python list, numpy ndarray or None
        """
        while self.is_capturing:
            try:
                yield self.data_queue.get_nowait()
            except queue.Empty:
                yield None

    def get_frame(self):
        """
        Fetches a signle frame of data from the RadarIQ sensor.

        :meth:`get_frame` should be used like this:

        .. code-block:: python

            while True:
                frame = get_frame():
                if frame is not None:
                    print(frame)

        The return type from :meth:`get_frame` is the same as :meth:`get_data`
        :return: A frame of data
        :rtype: a python list, numpy ndarray or None
        """
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None

    def _get_data_thread(self):
        mirror = 1 if self.mirror is False else -1  # multiplier for mirroring x-data
        rx_frame = []

        while self.is_capturing is True:
            try:
                subframe = self.connection.read_from_queue()
                if subframe is not None:
                    (command, variant) = unpack("<BB", subframe[:2])
                    if command == 0x68 and variant == 0x01:  # is statistics packet
                        self._process_statistics(subframe)

                    elif command == 0x66 and variant == 0x01:  # is a point cloud packet
                        (subframe_type, count) = unpack("<BB", subframe[2:4])

                        unpacking = "<" + "hhhB" * count
                        unpacked = unpack(unpacking, subframe[4:])
                        idx = 0
                        for cnt in range(count):
                            # SI units are needed so convert mm to m
                            rx_frame.append(
                                [mirror * units.convert_distance_from_si(self.distance_units, unpacked[idx] / 1000),
                                 units.convert_distance_from_si(self.distance_units,
                                                                unpacked[idx + 1] / 1000),
                                 units.convert_distance_from_si(self.distance_units,
                                                                unpacked[idx + 2] / 1000),
                                 unpacked[idx + 3]])
                            idx += 4

                        if subframe_type == 0x02:  # End of frame
                            self.capture_count += 1
                            if self.output_format == OUTPUT_LIST:  # @todo put tests around this
                                data = rx_frame
                            elif self.output_format == OUTPUT_NUMPY:
                                data = self._convert_pointcloud_to_numpy(rx_frame)
                            else:
                                data = None

                            try:
                                self.data_queue.put_nowait(data)
                            except queue.Full:
                                pass

                            rx_frame = []  # clear the buffer now the frame has been sent

                        if 0 < self.capture_max == self.capture_count:
                            break

                    elif command == 0x67 and variant == 0x01:  # is an object packet
                        (subframe_type, count) = unpack("<BB", subframe[2:4])
                        unpacking = "<" + "b9h" * count
                        unpacked = unpack(unpacking, subframe[4:])
                        idx = 0
                        for cnt in range(count):
                            # SI units are needed so convert mm to m

                            rx_frame.append(
                                {'tracking_id': unpacked[idx],
                                 'x_pos': mirror * units.convert_distance_from_si(self.distance_units,
                                                                                  unpacked[idx + 1] / 1000),
                                 'y_pos': units.convert_distance_from_si(self.distance_units,
                                                                         unpacked[idx + 2] / 1000),
                                 'z_pos': units.convert_distance_from_si(self.distance_units,
                                                                         unpacked[idx + 3] / 1000),
                                 'x_vel': mirror * units.convert_speed_from_si(self.speed_units,
                                                                               unpacked[idx + 4] / 1000),
                                 'y_vel': units.convert_speed_from_si(self.speed_units, unpacked[idx + 5] / 1000),
                                 'z_vel': units.convert_speed_from_si(self.speed_units, unpacked[idx + 6] / 1000),
                                 'x_acc': mirror * units.convert_acceleration_from_si(self.acceleration_units,
                                                                                      unpacked[idx + 7] / 1000),
                                 'y_acc': units.convert_acceleration_from_si(self.acceleration_units,
                                                                             unpacked[idx + 8] / 1000),
                                 'z_acc': units.convert_acceleration_from_si(self.acceleration_units,
                                                                             unpacked[idx + 9] / 1000)
                                 })
                            idx += 10

                        if subframe_type == 0x02:  # End of frame
                            self.capture_count += 1
                            if self.output_format == OUTPUT_LIST:  # @todo put tests around this
                                data = rx_frame
                            elif self.output_format == OUTPUT_NUMPY:
                                data = self._convert_object_tracking_to_numpy(rx_frame)
                            else:
                                data = None

                            try:
                                self.data_queue.put_nowait(data)
                            except queue.Full:
                                pass
                            rx_frame = []  # clear the buffer now the frame has been sent

                        if 0 < self.capture_max == self.capture_count:
                            break
                    else:
                        pass
                time.sleep(0.01)
            except ValueError:
                pass
        self.stop()

    def _convert_pointcloud_to_numpy(self, frame):
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

    def _convert_object_tracking_to_numpy(self, frame):
        """
        Convert the whole frame from a Python list to a numpy array
        :param frame: Frame to convert
        :return: Data as a numpy array [[tracking_id, xpos0, ypos0, zpos0, xvel0, yvel0, zvel0, xacc0, yacc0, zacc0]...]
        :rtype: ndarray
        """

        cnt = len(frame)
        data = np.zeros((cnt, 10))
        for idx, obj in enumerate(frame):
            data[idx][0] = obj["tracking_id"]
            data[idx][1] = obj["x_pos"]
            data[idx][2] = obj["y_pos"]
            data[idx][3] = obj["z_pos"]
            data[idx][4] = obj["x_vel"]
            data[idx][5] = obj["y_vel"]
            data[idx][6] = obj["z_vel"]
            data[idx][7] = obj["x_acc"]
            data[idx][8] = obj["y_acc"]
            data[idx][9] = obj["z_acc"]
        return data

    def _process_statistics(self, frame):
        """
        Process the statistics packet into a dictionary.
        
        :param frame: Frame of binary data
        :type frame: bytes
        """
        d = {}
        (d['active_frame_cpu'], d['inter_frame_cpu'], d['inter_frame_proc_time'], d['transmit_output_time'],
         d['inter_frame_proc_margin'], d['inter_chirp_proc_margin'], d['points_aggregation_time'],
         d['intensity_sort_time'], d['nearest_neighbours_time'], d['packet_transmit_time'],
         d['filter_points_removed'], d['num_transmitted_points'], d['input_points_truncated'],
         d['output_points_truncated'], d['temperature_sensor_0'], d['temperature_sensor_1'],
         d['temperature_power_management'], d['temperature_rx_0'], d['temperature_rx_1'],
         d['temperature_rx_2'], d['temperature_rx_3'], d['temperature_tx_0'], d['temperature_tx_1'],
         d['temperature_tx_2'],

         ) = unpack("<12L2B10h", frame[2:])

        d['rx_buffer_length'] = len(self.connection.rx_buffer)
        d['rx_packet_queue'] = self.get_queue_size()

        self.statistics = d

    def get_statistics(self):
        """
        Get the latest packet statistics.
        
        :return: Statistics about the sensor performance
        :rtype: dict
        """
        if self.statistics is not None:
            s = self.statistics.copy()
            self.statistics = None
            return s
        else:
            return None

    def get_queue_size(self):
        """
        Get the size of the data queue.

        This is the number of packets received but not yet processed.

        :return: The size of the queue
        :rtype: int
        """
        return self.data_queue.qsize()

    def _connection_status_callback(self, status):
        """
        Respond to the serial connection becoming disconnected.

        :param status: The serial connection status code
        """
        if status == CONNECTION_DISCONNECTED:
            self.is_capturing = False

        if self.application_connection_status_callback is not None:
            self.application_connection_status_callback(status)
