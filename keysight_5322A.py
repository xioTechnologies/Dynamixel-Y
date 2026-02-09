import pyvisa  # pip install pyvisa-py


class Keysight5322A:
    def __init__(self, gate_time: float) -> None:
        self.__session = pyvisa.ResourceManager("@py").open_resource("TCPIP::192.168.1.53::5025::SOCKET")
        self.__session.read_termination = "\n"

        self.__session.write("*RST")
        self.__session.write("INP1:FILT ON")
        self.__session.write(f"SENS:FREQ:GATE:TIME {gate_time}")

    def trigger(self) -> None:
        self.__session.write("INIT:IMM")

    def fetch(self) -> float:
        return float(self.__session.query("FETCH?").strip())
