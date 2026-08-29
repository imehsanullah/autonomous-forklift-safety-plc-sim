from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass

from pymodbus.client import ModbusTcpClient
from pymodbus.datastore import (
    ModbusDeviceContext,
    ModbusSequentialDataBlock,
    ModbusServerContext,
)
from pymodbus.server import ModbusTcpServer

from .model import SafetyPLC
from .registers import (
    DEFAULT_REGISTER_MAP,
    RegisterMap,
    decode_inputs,
    decode_outputs,
    encode_inputs,
    encode_outputs,
)
from .types import SafetyInputs, SafetyOutputs

LOG = logging.getLogger(__name__)


class ModbusSafetyClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 502,
        device_id: int = 1,
        register_map: RegisterMap = DEFAULT_REGISTER_MAP,
        timeout_s: float = 0.2,
    ) -> None:
        self.device_id = device_id
        self.register_map = register_map
        self.client = ModbusTcpClient(host, port=port, timeout=timeout_s, retries=1)

    def connect(self) -> bool:
        return bool(self.client.connect())

    def close(self) -> None:
        self.client.close()

    def exchange(self, inputs: SafetyInputs, write_inputs: bool = True) -> SafetyOutputs:
        if write_inputs:
            response = self.client.write_registers(
                self.register_map.input_address,
                encode_inputs(inputs),
                device_id=self.device_id,
            )
            if response.isError():
                raise ConnectionError(f"Modbus input write failed: {response}")

        response = self.client.read_holding_registers(
            self.register_map.output_address,
            count=self.register_map.output_count,
            device_id=self.device_id,
        )
        if response.isError():
            raise ConnectionError(f"Modbus output read failed: {response}")
        return decode_outputs(response.registers)


@dataclass(slots=True)
class EmulatorConfig:
    host: str = "127.0.0.1"
    port: int = 1502
    scan_period_s: float = 0.02
    device_id: int = 1


class PLCEmulator:
    def __init__(self, config: EmulatorConfig | None = None) -> None:
        self.config = config or EmulatorConfig()
        self.model = SafetyPLC()
        self._block = ModbusSequentialDataBlock(0, [0] * 4096)
        device = ModbusDeviceContext(hr=self._block)
        self._context = ModbusServerContext(devices={self.config.device_id: device}, single=False)
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._server: ModbusTcpServer | None = None
        self._ready = threading.Event()
        self._stop = threading.Event()
        self.last_exception: BaseException | None = None

    def start(self, timeout_s: float = 3.0) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._thread_main, name="plc-emulator", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout_s):
            raise TimeoutError("PLC emulator did not start")
        if self.last_exception:
            raise RuntimeError("PLC emulator failed") from self.last_exception

    def stop(self) -> None:
        self._stop.set()
        if self._loop and self._server:
            asyncio.run_coroutine_threadsafe(self._server.shutdown(), self._loop)
        if self._thread:
            self._thread.join(timeout=3.0)

    def __enter__(self) -> PLCEmulator:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()

    def _thread_main(self) -> None:
        try:
            asyncio.run(self._run())
        except BaseException as exc:
            self.last_exception = exc
            self._ready.set()

    async def _run(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._server = ModbusTcpServer(
            self._context,
            address=(self.config.host, self.config.port),
        )
        await self._server.serve_forever(background=True)
        self._ready.set()
        last_scan = time.monotonic()
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                values = self._context[self.config.device_id].getValues(
                    3,
                    DEFAULT_REGISTER_MAP.input_address,
                    count=DEFAULT_REGISTER_MAP.input_count,
                )
                inputs = decode_inputs(values)
                outputs = self.model.scan(inputs, now - last_scan)
                last_scan = now
                self._context[self.config.device_id].setValues(
                    3,
                    DEFAULT_REGISTER_MAP.output_address,
                    encode_outputs(outputs),
                )
                await asyncio.sleep(self.config.scan_period_s)
        finally:
            if self._server:
                await self._server.shutdown()
