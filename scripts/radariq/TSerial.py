import threading
import time
import six
import logging
from serial import Serial, LF, SerialException
from radariq.compatability import queue, pack, unpack, int_to_bytes

log = logging.getLogger('RadarIQ')

# Packet constants
PACKET_HEAD = 0xB0
PACKET_FOOT = 0xB1
PACKET_ESC = 0xB2
PACKET_XOR = 0x04
PACKET_HEAD_BYTES = int_to_bytes(PACKET_HEAD)
PACKET_FOOT_BYTES = int_to_bytes(PACKET_FOOT)
PACKET_ESC_BYTES = int_to_bytes(PACKET_ESC)
PACKET_XOR_BYTES = int_to_bytes(PACKET_XOR)

# Connection Statuses
CONNECTION_CONNECTED = 0
CONNECTION_DISCONNECTED = 1
CONNECTION_RECONNECTED = 2
CONNECTION_FATAL = 3


class TSerial(Serial):
    """
    Threaded serial implementation with enhancements.

    Supports:
    * Every command that pyserial supports
    * Faster 'read_until' implementation
    * Message passing via queues
    * Starting and stopping of threads

    Initialize just like a regular serial connection
    """

    def __init__(self, sleep=None, connection_status_callback=None, *args, **kwargs):
        """
        :param connection_status_callback: Callback to call when the connection status changes
        :param sleep: Number of seconds to sleep when run out of data (setting this will help lower CPU Usage)
            a warning will be issued to the log and data will be discarded (0 = unlimited).
        """
        self.sleep = sleep
        if connection_status_callback is not None:
            self.connection_status_callback = connection_status_callback
        else:
            self.connection_status_callback = self.noop

        super(TSerial, self).__init__(*args, **kwargs)

        self.last_comms = time.time()
        self.thread_running = False
        self._thread_process_running = False
        self.q = queue.Queue()
        self.rx_buffer = b''
        self.start()
        self.connection_status_callback(CONNECTION_CONNECTED)

    def flush_all(self):
        self.rx_buffer = b''
        self.flushInput()
        self.emtpy_queue()

    def start(self):
        """
        Start a receive thread running
        """
        processor = self._packet_rx

        self.thread_running = True
        if self._thread_process_running is False:
            t = threading.Thread(target=processor)
            t.start()

    def stop(self):
        """
        Stop a receive thread from running
        """
        self.thread_running = False
        time.sleep(0.1)

    def noop(self):
        """
        Function which does nothing. Used as default for callbacks
        """
        pass

    def _packet_rx(self):
        """
        Receive a packet from the serial stream and decode it and check the CRC.

        :return: Unescaped Packet with header, footer, crc removed
        :rtype: bytes
        """
        self._thread_process_running = True
        while self.thread_running:
            try:
                msg = self.read_fast(PACKET_FOOT_BYTES, PACKET_HEAD_BYTES)
                self.q.put_nowait(self.decode(msg))
            except queue.Full:
                log.warning("Cannot add message to the queue because the queue is full")
            except Exception as err:
                log.info(err)
        self._thread_process_running = False

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
        Works like read_until but is much faster and does not use timeouts or max sizes.

        This routine can optioally be made to sleep for short periods (to decrease CPU usage)

        :param footer: Footer byte to search for
        :param header: Header byte to search for (optional)
        :return: The first packet found in the rx buffer
        """
        while self.thread_running:
            try:
                self.rx_buffer += self.read_all()

                length = len(self.rx_buffer)
                start = 0
                for idx in range(length):
                    if header is not None and self.rx_buffer[idx:idx + 1] == header:
                        start = idx
                    if self.rx_buffer[idx:idx + 1] == footer:
                        msg = self.rx_buffer[start:idx + 1]
                        self.rx_buffer = self.rx_buffer[idx + 1:]
                        self.reset_comms_timer()
                        return msg

                if self.sleep is not None:
                    stay_awake = True if (time.time() - self.last_comms) < self.sleep else False
                    if stay_awake is False:
                        time.sleep(self.sleep)
            except SerialException:
                self._handle_disconnect()
            except Exception as ex:
                print(type(ex).__name__)

    def send_packet(self, msg):
        """
        Encode a message and send.

        :param msg: message to send
        :type msg: bytes
        :return: Number of bytes written
        :rtype: int
        """
        try:
            self.reset_comms_timer()
            return self.write(self.encode(msg))
        except SerialException:
            self._handle_disconnect()
        except Exception:
            log.warning("Failed to send data")

    def decode(self, src):
        """
        Decodes data from src.
        Expects header and footer bytes at the start and end of packet respectively.
        Unescapes any header, footer or escape bytes within the packet
        warning: Does not support messages over 255.

        :param src: Data to decode
        :type src: bytes
        :return: decoded data
        :rtype: bytes
        """
        if src[0:1] != PACKET_HEAD_BYTES:
            raise Exception("First byte of the message to decode is not a header byte")
        if src[-1:] != PACKET_FOOT_BYTES:
            raise Exception("Last byte of the message to decode is not a footer byte")
        dest = b""
        src_idx = 0

        # Loop through all bytes except footer
        while src_idx < len(src) - 1:
            char = src[src_idx:src_idx + 1]
            if char == PACKET_HEAD_BYTES:
                pass

            elif char == PACKET_ESC_BYTES:
                src_idx += 1  # Advance to the byte after the escape character
                char = src[src_idx:src_idx + 1]
                c = unpack("<B", char)[0]  # convert byts to int
                dest += int_to_bytes(c ^ PACKET_XOR)

            else:
                dest += char

            src_idx += 1

        # Crc check
        data = dest[0: -2]
        crc = self.crc16_ccitt(data)
        try:
            rx_crc = unpack("<H", dest[-2:])[0]
        except Exception:
            as_hex = ''.join(format(x, '02x') for x in src)
            raise Exception("Failed to extract CRC: {}".format(as_hex))
        if crc != rx_crc:
            as_hex = ''.join(format(x, '02x') for x in src)
            raise Exception("CRC Fail: {}".format(as_hex))
        else:
            return data

    def encode(self, src):
        """
        Encodes data from src.
        Adds header and footer to bytes to start and end of packet respectively.
        Escapes any header, footer or escape bytes.
        Does not support messages over 255.

        :param src Data needing to be encoded
        :type src: bytes
        :return: Encoded packet
        :rtype: bytes
        """

        # Add CRC to the source string (so it can be encoded)
        crc = self.crc16_ccitt(src)
        src += pack("<H", crc)
        src_idx = 0

        # Add packet header
        dest = PACKET_HEAD_BYTES
        # Loop through data, check for footer bytes in data and escape them
        while src_idx <= len(src):
            char = src[src_idx:src_idx + 1]
            if char == PACKET_HEAD_BYTES:
                dest += PACKET_ESC_BYTES
                dest += int_to_bytes(PACKET_HEAD ^ PACKET_XOR)

            elif char == PACKET_FOOT_BYTES:
                dest += PACKET_ESC_BYTES
                dest += int_to_bytes(PACKET_FOOT ^ PACKET_XOR)

            elif char == PACKET_ESC_BYTES:
                dest += PACKET_ESC_BYTES
                dest += int_to_bytes(PACKET_ESC ^ PACKET_XOR)

            else:
                dest += char
            src_idx += 1

        # Add the packet footer
        dest += PACKET_FOOT_BYTES

        if len(dest) > 255:
            raise Exception("Encoded packet is greater than the maximum of 255 bytes")

        return dest

    def reset_comms_timer(self):
        """
        Reset the timer which keeps track of when the last communications (inbound or outbound) occured.
        """
        self.last_comms = time.time()

    def crc16_ccitt(self, data):
        """
        CRC CCITT 0xFFFF algorithm.

        :param data: data to calculate the crc for
        :type data: bytes
        """
        crc = 0xffff
        msb = crc >> 8
        lsb = crc & 255
        for c in six.iterbytes(data):
            x = c ^ msb
            x ^= (x >> 4)
            msb = (lsb ^ (x >> 3) ^ (x << 4)) & 255
            lsb = (x ^ (x << 5)) & 255
        return (lsb << 8) + msb

    def _handle_disconnect(self):
        """
        Handle the disconnection of the serial connection.

        This will attempt auto-reconnection with back-off
        """
        self.close()
        self.connection_status_callback(CONNECTION_DISCONNECTED)

        for i in range(1, 10):
            try:
                log.error(
                    "The serial connection has been disconnected... attempting to reconnect. Attempt {}.".format(i))
                time.sleep(2)
                self.open()
                if self.is_open is True:
                    log.error("The serial connection has been restored.")
                    self.connection_status_callback(CONNECTION_RECONNECTED)
                    return
            except:
                pass
        log.error("The serial connection has been lost. Please manually reconnect.")
        self.connection_status_callback(CONNECTION_FATAL)
