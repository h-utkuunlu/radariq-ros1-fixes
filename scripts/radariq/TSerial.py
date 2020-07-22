import threading
import time
from radariq.compatability import queue, as_hex
import logging
from serial import Serial, LF
import radariq.LibBslPacket as bslpacket

log = logging.getLogger('RadarIQ')

# Modes
MODE_RAW = 0
MODE_BSL = 1


class TSerial(Serial):
    """
    Threaded serial implementation with enhancements.

    Supports:
    * Every command that pyserial supports
    * Faster 'read_until' implementation
    * Raw and BSL message formats
    * Message passing via queues
    * Starting and stopping of threads

    Initialize just like a regular serial connection
    """

    def __init__(self, mode=MODE_RAW, footer=LF, sleep=0.05, *args, **kwargs):
        """
        :param mode: The receive mode
        :param footer: Footer byte for packets
        :param sleep: Number of seconds to sleep when run out of data
                      (setting this will help lower CPU Usage) or None to disable
        """
        self.serial_mode = mode
        self.footer = footer
        self.sleep = sleep
        super(TSerial, self).__init__(*args, **kwargs)

        self.thread_running = False
        self.q = queue.Queue()
        self.rx_buffer = b''
        self.start()

    def flush_all(self):
        self.rx_buffer = b''
        self.flushInput()
        self.emtpy_queue()

    def start(self):
        """
        Start a receive thread running
        """
        if self.serial_mode == MODE_RAW:
            processor = self._raw_packet_rx
        elif self.serial_mode == MODE_BSL:
            processor = self._bsl_packet_rx
        else:
            raise Exception("Threaded Serial mode not supported")

        self.thread_running = True
        t = threading.Thread(target=processor)
        t.start()

    def stop(self):
        """
        Stop a receive thread from running
        """
        self.thread_running = False
        time.sleep(0.1)

    def _raw_packet_rx(self):
        """
        Receive a packet (terminated with footer byte) from the serial stream and decode it and check the CRC.

        :return: received message
        :rtype: bytes
        """
        while self.thread_running:
            try:
                msg = self.read_fast(self.footer)
                if msg is not None:
                    self.q.put_nowait(msg)
            except queue.Full:
                log.warning("Cannot add message to the queue because the queue is full")
            except Exception as err:
                log.info(err)

    def _bsl_packet_rx(self):
        """
        Receive a BSL packet from the serial stream and decode it and check the CRC.

        :return: Unescaped Packet with header, footer, crc removed
        :rtype: bytes
        """
        while self.thread_running:
            try:
                msg = self.read_fast(bslpacket.LIB_BSL_PACKET_FOOT_BYTES, bslpacket.LIB_BSL_PACKET_HEAD_BYTES)
                if msg is not None:
                    self.q.put_nowait(bslpacket.decode(msg))
            except queue.Full:
                log.warning("Cannot add message to the queue because the queue is full")
            except Exception as err:
                log.info(err)

    def read_from_queue(self):
        """
        Reads an item off the Queue.

        :return: A queued item (type dependent on the mode being used)
                 Or None if there were no items to fetch
        """
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None

    def emtpy_queue(self):
        """
        Empties the queue
        """
        with self.q.mutex:
            self.q.queue.clear()

    def read_fast(self, footer=LF, header=None):
        """
        Works like read_until but is much faster and does not use timeouts or max sizes
        :param footer: Footer byte to search for
        :param header: Header byte to search for (optional)
        :return: packet
        """
        try:
            while self.thread_running:
                self.rx_buffer += self.read_all()

                length = len(self.rx_buffer)
                start = 0
                for idx in range(length):
                    if header is not None and self.rx_buffer[idx:idx + 1] == header:
                        start = idx
                    if self.rx_buffer[idx:idx + 1] == footer:
                        msg = self.rx_buffer[start:idx + 1]
                        self.rx_buffer = self.rx_buffer[idx + 1:]
                        return msg
        except Exception as err:
            print(err)

        if self.sleep is not None:
            time.sleep(self.sleep)

    def send_bsl_packet(self, msg):
        """
        Encode a message with BSL packet and send.

        :param msg: message to send
        :type msg: bytes
        :return: Number of bytes written
        :rtype: int
        """
        try:
            return self.write(bslpacket.encode(msg))
        except Exception as err:
            log.warning(err)

    def send_bytes(self, msg):
        """
        Send a byte string to the serial.

        :param msg: message to send
        :type msg: bytes
        :return: Number of bytes written
        :rtype: int
        """
        return self.write(msg)

    def send_string(self, msg):
        """
        Send a string to the serial

        :param msg: message to send
        :type msg: str
        :return: Number of bytes written
        :rtype: int
        """
        return self.write(msg.encode('utf-8'))
