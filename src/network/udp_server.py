"""
UDP server that receives joint-angle updates in a background thread.

The server binds to a configurable host:port, receives datagrams,
parses them into JointUpdate objects, and stores the latest update
for the main thread to consume. Thread-safe via threading.Lock.
"""

import socket
import threading
import logging
from typing import Optional

from network.packet_parser import parse_packet, JointUpdate

logger = logging.getLogger(__name__)


class UDPServer:
    """
    Non-blocking UDP server running in a daemon thread.

    Usage:
        server = UDPServer('127.0.0.1', 9999)
        server.start()

        # In your main loop:
        update = server.get_latest_update()
        if update:
            robot.apply_update(update)

        # On shutdown:
        server.stop()
    """

    # Maximum UDP datagram size we accept
    MAX_PACKET_SIZE = 4096

    def __init__(self, host: str = '127.0.0.1', port: int = 9999):
        """
        Initialize the UDP server.

        Args:
            host: Interface to bind to. Default localhost.
            port: UDP port number. Default 9999.
        """
        self.host = host
        self.port = port
        self._socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._latest_update: Optional[JointUpdate] = None
        self._lock = threading.Lock()
        self._packet_count = 0
        self._error_count = 0

    def start(self):
        """Start the UDP server in a background daemon thread."""
        if self._running:
            logger.warning("UDP server is already running")
            return

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.host, self.port))
        # Timeout allows the receive loop to check the running flag
        self._socket.settimeout(0.1)

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

        logger.info(f"UDP server listening on {self.host}:{self.port}")

    def _receive_loop(self):
        """Receive loop running in the background thread."""
        while self._running:
            try:
                data, addr = self._socket.recvfrom(self.MAX_PACKET_SIZE)
                update = parse_packet(data)
                if update is not None:
                    with self._lock:
                        self._latest_update = update
                        self._packet_count += 1
                else:
                    self._error_count += 1
                    logger.debug(f"Malformed packet from {addr}")
            except socket.timeout:
                # Expected — just loop and check _running flag
                continue
            except OSError:
                # Socket was closed during shutdown
                if self._running:
                    logger.error("Socket error in receive loop")
                break
            except Exception as e:
                self._error_count += 1
                logger.error(f"Unexpected error in receive loop: {e}")

    def get_latest_update(self) -> Optional[JointUpdate]:
        """
        Get and consume the latest joint update (thread-safe).

        Returns:
            The most recent JointUpdate, or None if no new data.
        """
        with self._lock:
            update = self._latest_update
            self._latest_update = None
            return update

    def stop(self):
        """Stop the UDP server and clean up resources."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        logger.info(
            f"UDP server stopped. "
            f"Received {self._packet_count} valid packets, "
            f"{self._error_count} errors."
        )

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def stats(self) -> dict:
        """Server statistics."""
        return {
            'running': self._running,
            'packets_received': self._packet_count,
            'errors': self._error_count,
        }
