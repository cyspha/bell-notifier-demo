import socket, socketserver, http.server
import contextlib
import re
from time import time_ns

from playsound import playsound


#############
# CONSTANTS #
#############

# Server
SERVER_IP: str | None = None    # Set a host name string if you only want to allow connections from that host. Leave at None to allow connections from any host.
SERVER_PORT: int = 8001         # The port the server listens on.

# Bell notification
MIN_BELL_VOLTAGE: float = 1.0               # The minimum voltage received to trigger a notification sound.
MIN_BELL_INTERVAL_MS: float = 2.0 * 1000.0  # The minimum delay in milliseconds between sound notifications.
BELL_SOUND: str = './sound/test.wav'        # The sound to play when the voltage received is above the threshold.

# Debug
DO_DEBUG_PRINT = True


##########
# SERVER #
##########

class MyTCPHandler(socketserver.BaseRequestHandler):

    """Handles incoming TCP data.

    """

    last_bell_notify_timestamp_ms: int = 0

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print("{} sent message: {}".format(self.client_address[0], self.data))

        formatted_data = self.format_data(self.data)

        self.do_bell_ringing_notification(formatted_data['v_adc'])
            
        # just send back the same data, but upper-cased
        # self.request.sendall(self.data.upper())
        self.request.sendall(bytes('OK', 'ascii'))


    def format_data(self, data: bytes) -> dict[str]:
        """Formats received TCP data into an easily usable dict."""
        formatted_data: dict[str] = {}

        # adc voltage
        adc_match = re.search("(?<=adc=)\d+\.?\d*", str(data))
        formatted_data['v_adc'] = float(adc_match.group()) if adc_match != None else float('-inf')

        return formatted_data


    def do_bell_ringing_notification(self, voltage: float):
        if type(self.server) != DoorbellTCPServer:
            return
        timestamp = self.server.last_bell_notify_timestamp_ms

        debug_print("  > ADC voltage = {} V (Threshold: {} V)".format(voltage, MIN_BELL_VOLTAGE))

        should_ring = (voltage >= MIN_BELL_VOLTAGE) and (time_ms() > timestamp + MIN_BELL_INTERVAL_MS)
        if should_ring:
            self.server.last_bell_notify_timestamp_ms = time_ms()
            playsound(BELL_SOUND)


class DoorbellTCPServer(socketserver.TCPServer):

    """Overrides base TCPServer to allow both IPv4 and IPv6 connections

    """

    address_family, server_address = http.server._get_best_family(SERVER_IP, SERVER_PORT)


    def __init__(self, server_address, request_handler_class, bind_and_activate: bool = ...) -> None:
        self.last_bell_notify_timestamp_ms: int = 0
        super().__init__(server_address, request_handler_class, bind_and_activate)


    def server_bind(self) -> None:
        with contextlib.suppress(Exception):
            self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        return super().server_bind()


###########
# HELPERS #
###########

def time_ms() -> int:
    return time_ns() // (1000 * 1000)


def debug_print(*args):
    if DO_DEBUG_PRINT:
        print(args)


########
# MAIN #
########
if __name__ == "__main__":
    server = DoorbellTCPServer(DoorbellTCPServer.server_address, MyTCPHandler)
    print("Starting BellNotifier TCP Server on address {} and port {}.".format(server.server_address, SERVER_PORT))
    server.serve_forever()