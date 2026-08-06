"""只读 Modbus RTU 编码器高度源。"""

from collections.abc import Callable
from inspect import signature
from time import monotonic
from typing import Any

from .config import SensorConfig
from .types import HeightSample

MAX_RAW_COUNT = 61440


class ModbusRtuHeightSource:
    """以 FC03 读取两个保持寄存器的轮式编码器高度源。"""

    def __init__(
        self,
        config: SensorConfig,
        *,
        client_factory: Callable[..., Any] | None = None,
        clock: Callable[[], float] = monotonic,
    ) -> None:
        self._config = config
        self._client_factory = client_factory or _create_rtu_client
        self._clock = clock
        self._client: Any | None = None
        self._connected = False
        self._unit_parameter: str | None = None

    def open(self) -> bool:
        """创建并连接串口客户端；失败由返回值表示，便于上层决定重试策略。"""
        if self._connected:
            return True
        client: Any | None = None
        try:
            client = self._client_factory(
                port=self._config.port,
                baudrate=self._config.baudrate,
                bytesize=self._config.bytesize,
                parity=self._config.parity,
                stopbits=self._config.stopbits,
                timeout=self._config.timeout_s,
            )
            if not client.connect():
                self._close_client(client)
                return False
            unit_parameter = self._detect_unit_parameter(client.read_holding_registers)
            self._client = client
            self._unit_parameter = unit_parameter
            self._connected = True
        except Exception:
            self._close_client(client)
            self._client = None
            self._connected = False
            self._unit_parameter = None
        return self._connected

    def read_sample(self) -> HeightSample:
        """读取 FC03 数据，并将任何通信或协议错误转换为无效样本。"""
        timestamp = self._clock()
        if not self._connected or self._client is None:
            return self._invalid(timestamp, "Modbus RTU 未连接")
        try:
            response = self._read_holding_registers()
        except Exception as exc:
            self.close()
            return self._invalid(timestamp, f"Modbus RTU 读取失败: {exc}")
        if response is None:
            return self._invalid(timestamp, "Modbus RTU 响应为空")
        try:
            if response.isError():
                return self._invalid(timestamp, "Modbus RTU 返回错误响应")
            registers = response.registers
        except Exception as exc:
            return self._invalid(timestamp, f"Modbus RTU 响应协议无效: {exc}")
        if not isinstance(registers, (list, tuple)) or len(registers) != 2:
            return self._invalid(timestamp, "Modbus RTU 响应寄存器数量不是 2")
        if any(type(word) is not int or not 0 <= word <= 0xFFFF for word in registers):
            return self._invalid(timestamp, "Modbus RTU 响应包含无效 16 位寄存器")

        # 两个 16 位寄存器组装为无符号 32 位计数；设备可配置高字在前或低字在前。
        first, second = registers
        if self._config.word_order == "high_low":
            raw_count = (first << 16) | second
        else:
            raw_count = (second << 16) | first
        if not 0 <= raw_count <= MAX_RAW_COUNT:
            return self._invalid(timestamp, f"编码器计数超出范围 (out of range) 0..{MAX_RAW_COUNT}", raw_count)

        # 位移换算以轮周长/每圈计数为比例；range_mm 是安装行程的二次安全上限。
        height_mm = raw_count * self._config.wheel_circumference_mm / self._config.counts_per_revolution
        if height_mm > self._config.range_mm:
            return self._invalid(timestamp, "换算高度超出配置量程", raw_count, height_mm)
        return HeightSample(timestamp, raw_count, height_mm, True, None)

    def close(self) -> None:
        """幂等关闭连接；本模块从不写入传感器零点、模式或任何寄存器。"""
        client, self._client = self._client, None
        self._connected = False
        self._unit_parameter = None
        self._close_client(client)

    def _read_holding_registers(self) -> Any:
        """使用打开连接时缓存的站号参数，只调用一次底层读方法。"""
        assert self._client is not None
        assert self._unit_parameter is not None
        method = self._client.read_holding_registers
        arguments = {
            "address": self._config.register_address,
            "count": self._config.register_count,
        }
        arguments[self._unit_parameter] = self._config.slave_id
        return method(**arguments)

    @staticmethod
    def _detect_unit_parameter(method: Callable[..., Any]) -> str:
        """打开连接后解析一次 pymodbus 签名，兼容新版 device_id 与旧版 slave。"""
        parameters = signature(method).parameters
        # 只依据签名决定参数名，不捕获 TypeError 后重试，以保留底层通信异常语义。
        if "device_id" in parameters:
            return "device_id"
        if "slave" in parameters:
            return "slave"
        raise TypeError("read_holding_registers 缺少 device_id 或 slave 参数")

    @staticmethod
    def _close_client(client: Any | None) -> None:
        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    @staticmethod
    def _invalid(
        timestamp: float,
        error: str,
        raw_count: int | None = None,
        height_mm: float | None = None,
    ) -> HeightSample:
        return HeightSample(timestamp, raw_count, height_mm, False, error)


def _create_rtu_client(**kwargs: Any) -> Any:
    """延迟导入 pymodbus，避免纯单元测试依赖真实串口驱动。"""
    from pymodbus.client import ModbusSerialClient

    return ModbusSerialClient(**kwargs)
