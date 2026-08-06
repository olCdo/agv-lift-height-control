import inspect

import agv_lift_height_control.modbus_rtu as rtu_module
from agv_lift_height_control import ModbusRtuHeightSource, SensorConfig


class FakeResponse:
    def __init__(self, registers=None, error: bool = False) -> None:
        self.registers = registers
        self._error = error

    def isError(self) -> bool:
        return self._error


class FakeClient:
    def __init__(self, *, connect_result: bool = True, response=None) -> None:
        self.connect_result = connect_result
        self.response = response
        self.connected = False
        self.closed = 0
        self.calls: list[dict[str, int]] = []

    def connect(self) -> bool:
        self.connected = self.connect_result
        return self.connect_result

    def read_holding_registers(self, *, address: int, count: int, slave: int):
        self.calls.append({"address": address, "count": count, "slave": slave})
        if isinstance(self.response, Exception):
            raise self.response
        return self.response

    def close(self) -> None:
        self.closed += 1
        self.connected = False


class FakeDeviceIdClient(FakeClient):
    def read_holding_registers(self, *, address: int, count: int, device_id: int):
        self.calls.append({"address": address, "count": count, "device_id": device_id})
        if isinstance(self.response, Exception):
            raise self.response
        return self.response


class ClientFactory:
    def __init__(self, *clients: FakeClient) -> None:
        self._clients = list(clients)
        self.calls = 0

    def __call__(self, **_):
        self.calls += 1
        return self._clients.pop(0)


class SensorConfigProbe:
    def __init__(self, config: SensorConfig) -> None:
        self._config = config
        self.register_count_reads = 0

    @property
    def register_count(self) -> int:
        self.register_count_reads += 1
        return self._config.register_count

    def __getattr__(self, name: str):
        return getattr(self._config, name)


def sensor_config(**changes) -> SensorConfig:
    data = {
        "transport": "rtu",
        "port": "/dev/ttyS7",
        "baudrate": 115200,
        "bytesize": 8,
        "parity": "N",
        "stopbits": 1,
        "timeout_s": 0.1,
        "slave_id": 3,
        "register_address": 0,
        "register_count": 2,
        "word_order": "high_low",
        "wheel_circumference_mm": 200.0,
        "counts_per_revolution": 4096,
        "range_mm": 3000.0,
        "poll_period_s": 0.02,
    }
    data.update(changes)
    return SensorConfig(**data)


def test_read_sample_combines_high_low_words_and_scales_to_mm() -> None:
    client = FakeClient(response=FakeResponse([0x0000, 0x0800]))
    source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client, clock=lambda: 4.25)

    assert source.open() is True
    sample = source.read_sample()

    assert sample.raw_count == 2048
    assert sample.height_mm == 100.0
    assert sample.timestamp == 4.25
    assert sample.valid is True
    assert sample.error is None
    assert client.calls == [{"address": 0, "count": 2, "slave": 3}]


def test_read_sample_supports_low_high_word_order_and_configured_modbus_target() -> None:
    client = FakeClient(response=FakeResponse([0x0800, 0x0000]))
    source = ModbusRtuHeightSource(
        sensor_config(word_order="low_high", register_address=12, slave_id=9),
        client_factory=lambda **_: client,
        clock=lambda: 1.0,
    )
    source.open()

    sample = source.read_sample()

    assert sample.valid is True
    assert sample.raw_count == 2048
    assert client.calls == [{"address": 12, "count": 2, "slave": 9}]


def test_read_sample_uses_device_id_for_newer_pymodbus_signature() -> None:
    client = FakeDeviceIdClient(response=FakeResponse([0x0000, 0x0800]))
    source = ModbusRtuHeightSource(
        sensor_config(register_address=12, slave_id=9),
        client_factory=lambda **_: client,
        clock=lambda: 1.0,
    )
    source.open()

    sample = source.read_sample()

    assert sample.valid is True
    assert client.calls == [{"address": 12, "count": 2, "device_id": 9}]


def test_read_sample_reads_register_count_from_config_object() -> None:
    client = FakeClient(response=FakeResponse([0x0000, 0x0000]))
    config = SensorConfigProbe(sensor_config())
    source = ModbusRtuHeightSource(
        config,  # type: ignore[arg-type]
        client_factory=lambda **_: client,
        clock=lambda: 1.0,
    )
    source.open()

    source.read_sample()

    assert config.register_count_reads == 1
    assert client.calls == [{"address": 0, "count": 2, "slave": 3}]


def test_read_sample_returns_invalid_samples_for_error_and_bad_responses() -> None:
    cases = [
        FakeResponse(error=True),
        FakeResponse([1]),
        FakeResponse([1, 2, 3]),
        FakeResponse([1, 65536]),
        FakeClient(response=RuntimeError("serial down")),
    ]
    for case in cases:
        client = case if isinstance(case, FakeClient) else FakeClient(response=case)
        source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client, clock=lambda: 2.0)
        source.open()

        sample = source.read_sample()

        assert sample.valid is False
        assert sample.error
        assert sample.timestamp == 2.0


def test_read_sample_rejects_out_of_range_count() -> None:
    client = FakeClient(response=FakeResponse([0x0001, 0x0000]))
    source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client, clock=lambda: 2.0)
    source.open()

    sample = source.read_sample()

    assert sample.valid is False
    assert sample.raw_count == 65536
    assert "range" in (sample.error or "")


def test_read_sample_accepts_the_inclusive_maximum_count() -> None:
    client = FakeClient(response=FakeResponse([0x0000, 0xF000]))
    source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client, clock=lambda: 2.0)
    source.open()

    sample = source.read_sample()

    assert sample.valid is True
    assert sample.raw_count == 61440
    assert sample.height_mm == 3000.0


def test_open_connection_failure_and_read_before_open_are_safe() -> None:
    client = FakeClient(connect_result=False, response=FakeResponse([0, 0]))
    source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client, clock=lambda: 3.0)

    assert source.read_sample().valid is False
    assert source.open() is False
    assert source.read_sample().valid is False


def test_open_connection_failure_closes_created_client() -> None:
    client = FakeClient(connect_result=False, response=FakeResponse([0, 0]))
    source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client)

    assert source.open() is False
    assert client.closed == 1


def test_read_exception_closes_client_and_allows_open_to_reconnect() -> None:
    failed_client = FakeClient(response=RuntimeError("serial down"))
    recovered_client = FakeClient(response=FakeResponse([0, 0]))
    factory = ClientFactory(failed_client, recovered_client)
    source = ModbusRtuHeightSource(sensor_config(), client_factory=factory, clock=lambda: 5.0)

    assert source.open() is True
    assert source.read_sample().valid is False
    assert failed_client.closed == 1

    assert source.open() is True
    assert factory.calls == 2
    assert source.read_sample().valid is True


def test_open_caches_unit_parameter_signature_for_multiple_samples(monkeypatch) -> None:
    client = FakeDeviceIdClient(response=FakeResponse([0, 0]))
    calls = 0

    def tracked_signature(method):
        nonlocal calls
        calls += 1
        return inspect.signature(method)

    monkeypatch.setattr(rtu_module, "signature", tracked_signature)
    source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client, clock=lambda: 1.0)

    assert source.open() is True
    assert source.read_sample().valid is True
    assert source.read_sample().valid is True
    assert calls == 1


def test_close_is_idempotent() -> None:
    client = FakeClient(response=FakeResponse([0, 0]))
    source = ModbusRtuHeightSource(sensor_config(), client_factory=lambda **_: client)
    source.open()

    source.close()
    source.close()

    assert client.closed == 1
