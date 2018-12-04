import errno
import threading
from six.moves import queue
import socket
import time

# Audio recording parameters
RATE = 16000
CHUNK = 1600


class AudioServerStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate=RATE, chunk=CHUNK, server_name='127.0.0.1',
                 port=4444):
        self._rate = rate
        self._chunk = chunk
        self._server_name = server_name
        self._port = port

        # Socket for connection
        self.s = None
        self._connected = False

        # Audio data thread to get data from server
        self.data_thread = threading.Thread(target=self._get_server_data)
        self.data_thread.daemon = True

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def _connect(self):
        """Creates a socket to listen for audio data from the server."""
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self._server_name, self._port))
        self._connected = True

    def _get_server_data(self):
        """Daemon thread that receives data from the audio socket and puts in a
        buffer. Works just like _get_audio_data but data comes from server,
        not mic.
        """
        try:
            while True:
                data = self.s.recv(self._chunk)
                self._buff.put(data)
        except KeyboardInterrupt as e:
            print("AUDIO_SERVER: Shutdown from thread: {}".format(e))
            self.__exit__()

    def __enter__(self):
        """Makes 3 attempts at connecting to the audio server defined in the
        parameters file.
        """
        print("AUDIO_SERVER: Using audio server.")
        # Retry 3 times to connect
        MAX_CONNECTION_RETRY = 3
        for _ in range(0, MAX_CONNECTION_RETRY):
            try:
                self._connect()
            except socket.error as e:
                print("AUDIO_SERVER: Socket exception caught!\n{}\n"
                      "Retrying...".format(e))
                time.sleep(1)
                continue
            break
        # Yay :)
        if self._connected:
            print("AUDIO_SERVER: Connected to audio server.")
            self.data_thread.start()
            self.closed = False
            return self
        # Nay :c
        else:
            raise errno.ECONNREFUSED("AUDIO_SERVER: Unable to connect to audio "
                                     "server! Make sure it is running and you "
                                     "are connected on the same network.")

    def __exit__(self, type, value, traceback):
        self.data_thread.join()
        self.s.shutdown()
        self.s.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)
