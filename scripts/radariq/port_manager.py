from serial import Serial, SerialException
from serial.tools import list_ports

"""
Find the COM port(s) RadarIQ modules are connected
"""

USB_VID = 5840
USB_PID = 3797


def find_com_port():
    """
    Search for one RadarIQ module.

    This return the first RadarIQ module found that is not currently in use.

    *Usage:*

    .. code-block:: python

        from radariq import find_com_port

        port = find_com_port()
        print(port.device)

    :return: The port the RadarIQ module is connected to.
    :rtype: ListPortInfo
    """
    try:
        ports = find_com_ports()
        return ports[0]
    except Exception as err:
        raise Exception("No RadarIQ modules detected")


def find_com_ports():
    """
    Find the COM port(s) of RadarIQ modules which are not currently in use.


    *Usage:*

    .. code-block:: python

        from radariq import find_com_ports()

        ports = find_com_ports()
        for port in ports:
            print(port.device)


    :return: A list of RadarIQ modules.
    :rtype: a list of ListPortInfo objects
    """
    raw_ports = list_ports.comports()

    ports = []

    for port in raw_ports:
        try:
            if port.vid == USB_VID and port.pid == USB_PID:
                connection = Serial(port=port.device, baudrate=115200, timeout=1)
                connection.close()
                ports.append(port)
        except SerialException:
            # Probably in use
            pass

    if len(ports) == 0:
        raise Exception("No available RadarIQ modules detected")
    else:
        return ports
