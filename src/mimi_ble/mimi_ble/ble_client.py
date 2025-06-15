import asyncio
from bleak import BleakClient, BleakGATTCharacteristic, BleakError
from dbus_next.aio import MessageBus
from dbus_next.constants import BusType
from dbus_next import Variant

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_WRITE_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
UART_READ_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


class BleClient:
    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        self.client: BleakClient | None = None
        self._rx_callback = None

        self._tx_queue = asyncio.Queue()
        self._tx_task: asyncio.Task | None = None

    def set_rx_callback(self, callback):
        """Установить обработчик входящих строк (str)."""
        self._rx_callback = callback

    async def _find_robot_address(self):
        bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        introspection = await bus.introspect("org.bluez", "/")
        obj = bus.get_proxy_object("org.bluez", "/", introspection)
        manager = obj.get_interface("org.freedesktop.DBus.ObjectManager")
        # noinspection PyUnresolvedReferences
        objects = await manager.call_get_managed_objects()

        for path, interfaces in objects.items():
            if "org.bluez.Device1" in interfaces:
                props = interfaces["org.bluez.Device1"]
                alias = props.get("Alias", Variant("s", "")).value
                address = props.get("Address", Variant("s", "")).value
                paired = props.get("Paired", Variant("b", False)).value
                if alias == self.robot_name and paired:
                    return address
        return None

    async def start(self):
        address = await self._find_robot_address()
        if not address:
            raise RuntimeError(f"Robot '{self.robot_name}' not found among paired devices.")

        self.client = BleakClient(address)
        await self.client.connect()

        if not self.client.is_connected:
            raise RuntimeError("BLE connection failed.")

        await self.client.start_notify(UART_READ_UUID, self._on_rx)

        self._tx_task = asyncio.create_task(self._tx_worker())

    async def stop(self):
        if self._tx_task:
            await self._tx_queue.put(None)
            await self._tx_task
            self._tx_task = None

        if self.client and self.client.is_connected:
            await self.client.stop_notify(UART_READ_UUID)
            await self.client.disconnect()

    def _on_rx(self, _: BleakGATTCharacteristic, data: bytearray):
        if self._rx_callback:
            message = data.decode(errors="ignore").strip()
            self._rx_callback(message)

    async def send(self, text: str):
        """Асинхронно добавить сообщение в очередь на отправку."""
        await self._tx_queue.put(text)

    async def _tx_worker(self):
        """Обрабатывает очередь и отправляет сообщения последовательно."""
        while True:
            text = await self._tx_queue.get()
            if text is None:
                break
            if self.client and self.client.is_connected:
                if not text.endswith("\n"):
                    text += "\n"
                try:
                    await self.client.write_gatt_char(UART_WRITE_UUID, text.encode())
                except Exception as e:
                    print(f"⚠️ Failed to send over BLE: {e}")
            self._tx_queue.task_done()
