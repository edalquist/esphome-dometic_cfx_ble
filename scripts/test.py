import asyncio
from bleak import BleakScanner, BleakClient
import struct

SERVICE_UUID = "537a0300-0995-481f-926c-1604e23fd515"
WRITE_UUID = "537a0301-0995-481f-926c-1604e23fd515"
NOTIFY_UUID = "537a0302-0995-481f-926c-1604e23fd515"

ACTION_PUB = 0
ACTION_SUB = 1
ACTION_PING = 2
ACTION_HELLO = 3
ACTION_ACK = 4
ACTION_NAK = 5
ACTION_NOP = 6

TOPICS = {
    "SUBSCRIBE_APP_SZ": {"param": [1,0,0,129], "type": "EMPTY", "desc": "Subscribe all SZ"},
    "SUBSCRIBE_APP_SZI": {"param": [2,0,0,129], "type": "EMPTY", "desc": "Subscribe all SZI"},
    "SUBSCRIBE_APP_DZ": {"param": [3,0,0,129], "type": "EMPTY", "desc": "Subscribe all DZ"},
    "PRODUCT_SERIAL_NUMBER": {"param": [0,193,0,0], "type": "UTF8_STRING", "desc": "Serial number"},
    "COMPARTMENT_COUNT": {"param": [0,128,0,1], "type": "INT8_NUMBER", "desc": "Compartments count"},
    "ICEMAKER_COUNT": {"param": [0,129,0,1], "type": "INT8_NUMBER", "desc": "Icemakers count"},
    "COMPARTMENT_0_POWER": {"param": [0,0,1,1], "type": "INT8_BOOLEAN", "desc": "Compartment 1 power"},
    "COMPARTMENT_1_POWER": {"param": [16,0,1,1], "type": "INT8_BOOLEAN", "desc": "Compartment 2 power"},
    "COMPARTMENT_0_MEASURED_TEMPERATURE": {"param": [0,1,1,1], "type": "INT16_DECIDEGREE_CELSIUS", "desc": "Compartment 1 current temp"},
    "COMPARTMENT_1_MEASURED_TEMPERATURE": {"param": [16,1,1,1], "type": "INT16_DECIDEGREE_CELSIUS", "desc": "Compartment 2 current temp"},
    "COMPARTMENT_0_DOOR_OPEN": {"param": [0,8,1,1], "type": "INT8_BOOLEAN", "desc": "Compartment 1 door open"},
    "COMPARTMENT_1_DOOR_OPEN": {"param": [16,8,1,1], "type": "INT8_BOOLEAN", "desc": "Compartment 2 door open"},
    "COMPARTMENT_0_SET_TEMPERATURE": {"param": [0,2,1,1], "type": "INT16_DECIDEGREE_CELSIUS", "desc": "Compartment 1 set temp"},
    "COMPARTMENT_1_SET_TEMPERATURE": {"param": [16,2,1,1], "type": "INT16_DECIDEGREE_CELSIUS", "desc": "Compartment 2 set temp"},
    "COMPARTMENT_0_RECOMMENDED_RANGE": {"param": [0,129,1,1], "type": "INT16_ARRAY", "desc": "Compartment 1 recommended range"},
    "COMPARTMENT_1_RECOMMENDED_RANGE": {"param": [16,129,1,1], "type": "INT16_ARRAY", "desc": "Compartment 2 recommended range"},
    "PRESENTED_TEMPERATURE_UNIT": {"param": [0,0,2,1], "type": "INT8_NUMBER", "desc": "Temperature unit"},
    "COMPARTMENT_0_TEMPERATURE_RANGE": {"param": [0,128,1,1], "type": "INT16_ARRAY", "desc": "Compartment 1 allowed range"},
    "COMPARTMENT_1_TEMPERATURE_RANGE": {"param": [16,128,1,1], "type": "INT16_ARRAY", "desc": "Compartment 2 allowed range"},
    "COOLER_POWER": {"param": [0,0,3,1], "type": "INT8_BOOLEAN", "desc": "Cooler power"},
    "BATTERY_VOLTAGE_LEVEL": {"param": [0,1,3,1], "type": "INT16_DECICURRENT_VOLT", "desc": "Battery voltage"},
    "BATTERY_PROTECTION_LEVEL": {"param": [0,2,3,1], "type": "UINT8_NUMBER", "desc": "Battery protection level"},
    "POWER_SOURCE": {"param": [0,5,3,1], "type": "INT8_NUMBER", "desc": "Power source"},
    "ICEMAKER_POWER": {"param": [0,6,3,1], "type": "INT8_BOOLEAN", "desc": "Icemaker power"},
    "COMMUNICATION_ALARM": {"param": [0,3,4,1], "type": "INT8_BOOLEAN", "desc": "Communication alarm"},
    "NTC_OPEN_LARGE_ERROR": {"param": [0,1,4,1], "type": "INT8_BOOLEAN", "desc": "NTC open large"},
    "NTC_SHORT_LARGE_ERROR": {"param": [0,2,4,1], "type": "INT8_BOOLEAN", "desc": "NTC short large"},
    "SOLENOID_VALVE_ERROR": {"param": [0,9,4,1], "type": "INT8_BOOLEAN", "desc": "Solenoid valve error"},
    "NTC_OPEN_SMALL_ERROR": {"param": [0,17,4,1], "type": "INT8_BOOLEAN", "desc": "NTC open small"},
    "NTC_SHORT_SMALL_ERROR": {"param": [0,18,4,1], "type": "INT8_BOOLEAN", "desc": "NTC short small"},
    "FAN_OVERVOLTAGE_ERROR": {"param": [0,50,4,1], "type": "INT8_BOOLEAN", "desc": "Fan overvoltage"},
    "COMPRESSOR_START_FAIL_ERROR": {"param": [0,51,4,1], "type": "INT8_BOOLEAN", "desc": "Compressor start fail"},
    "COMPRESSOR_SPEED_ERROR": {"param": [0,52,4,1], "type": "INT8_BOOLEAN", "desc": "Compressor speed error"},
    "CONTROLLER_OVER_TEMPERATURE": {"param": [0,53,4,1], "type": "INT8_BOOLEAN", "desc": "Controller over temp"},
    "TEMPERATURE_ALERT_DCM": {"param": [0,3,5,1], "type": "INT8_BOOLEAN", "desc": "Temp alert DCM"},
    "TEMPERATURE_ALERT_CC": {"param": [0,0,5,1], "type": "INT8_BOOLEAN", "desc": "Temp alert CC"},
    "DOOR_ALERT": {"param": [0,1,5,1], "type": "INT8_BOOLEAN", "desc": "Door alert"},
    "VOLTAGE_ALERT": {"param": [0,2,5,1], "type": "INT8_BOOLEAN", "desc": "Voltage alert"},
    "DEVICE_NAME": {"param": [0,0,6,1], "type": "UTF8_STRING", "desc": "Device name"},
    "WIFI_MODE": {"param": [0,1,6,1], "type": "INT8_BOOLEAN", "desc": "WiFi mode"},
    "BLUETOOTH_MODE": {"param": [0,3,6,1], "type": "INT8_BOOLEAN", "desc": "Bluetooth mode"},
    "WIFI_AP_CONNECTED": {"param": [0,8,6,1], "type": "INT8_BOOLEAN", "desc": "WiFi AP connected"},
    "STATION_SSID_0": {"param": [0,0,7,1], "type": "UTF8_STRING", "desc": "Station SSID 0"},
    "STATION_SSID_1": {"param": [1,0,7,1], "type": "UTF8_STRING", "desc": "Station SSID 1"},
    "STATION_SSID_2": {"param": [2,0,7,1], "type": "UTF8_STRING", "desc": "Station SSID 2"},
    "STATION_PASSWORD_0": {"param": [0,1,7,1], "type": "UTF8_STRING", "desc": "Station password 0"},
    "STATION_PASSWORD_1": {"param": [1,1,7,1], "type": "UTF8_STRING", "desc": "Station password 1"},
    "STATION_PASSWORD_2": {"param": [2,1,7,1], "type": "UTF8_STRING", "desc": "Station password 2"},
    "STATION_PASSWORD_3": {"param": [3,1,7,1], "type": "UTF8_STRING", "desc": "Station password 3"},
    "STATION_PASSWORD_4": {"param": [4,1,7,1], "type": "UTF8_STRING", "desc": "Station password 4"},
    "CFX_DIRECT_PASSWORD_0": {"param": [0,2,7,1], "type": "UTF8_STRING", "desc": "CFX direct password 0"},
    "CFX_DIRECT_PASSWORD_1": {"param": [1,2,7,1], "type": "UTF8_STRING", "desc": "CFX direct password 1"},
    "CFX_DIRECT_PASSWORD_2": {"param": [2,2,7,1], "type": "UTF8_STRING", "desc": "CFX direct password 2"},
    "CFX_DIRECT_PASSWORD_3": {"param": [3,2,7,1], "type": "UTF8_STRING", "desc": "CFX direct password 3"},
    "CFX_DIRECT_PASSWORD_4": {"param": [4,2,7,1], "type": "UTF8_STRING", "desc": "CFX direct password 4"},
    "COMPARTMENT_0_TEMPERATURE_HISTORY_HOUR": {"param": [0,64,1,1], "type": "HISTORY_DATA_ARRAY", "desc": "Comp 1 hour temp history"},
    "COMPARTMENT_1_TEMPERATURE_HISTORY_HOUR": {"param": [16,64,1,1], "type": "HISTORY_DATA_ARRAY", "desc": "Comp 2 hour temp history"},
    "COMPARTMENT_0_TEMPERATURE_HISTORY_DAY": {"param": [0,65,1,1], "type": "HISTORY_DATA_ARRAY", "desc": "Comp 1 day temp history"},
    "COMPARTMENT_1_TEMPERATURE_HISTORY_DAY": {"param": [16,65,1,1], "type": "HISTORY_DATA_ARRAY", "desc": "Comp 2 day temp history"},
    "COMPARTMENT_0_TEMPERATURE_HISTORY_WEEK": {"param": [0,66,1,1], "type": "HISTORY_DATA_ARRAY", "desc": "Comp 1 week temp history"},
    "COMPARTMENT_1_TEMPERATURE_HISTORY_WEEK": {"param": [16,66,1,1], "type": "HISTORY_DATA_ARRAY", "desc": "Comp 2 week temp history"},
    "DC_CURRENT_HISTORY_HOUR": {"param": [0,64,3,1], "type": "HISTORY_DATA_ARRAY", "desc": "DC current hour history"},
    "DC_CURRENT_HISTORY_DAY": {"param": [0,65,3,1], "type": "HISTORY_DATA_ARRAY", "desc": "DC current day history"},
    "DC_CURRENT_HISTORY_WEEK": {"param": [0,66,3,1], "type": "HISTORY_DATA_ARRAY", "desc": "DC current week history"},
}

ACTION_NAMES = {
    ACTION_PUB: "PUB",
    ACTION_SUB: "SUB",
    ACTION_PING: "PING",
    ACTION_HELLO: "HELLO",
    ACTION_ACK: "ACK",
    ACTION_NAK: "NAK",
    ACTION_NOP: "NOP",
}

BATTERY_LEVELS = {0: "Low", 1: "Medium", 2: "High"}
POWER_SOURCES = {0: "AC", 1: "DC", 2: "Solar"}
NO_VALUE = -3276.8

def encode_value(value, data_type):
    if data_type == "INT16_DECIDEGREE_CELSIUS":
        deci = int(round(value * 10))
        return struct.pack("<h", deci)
    elif data_type == "INT8_BOOLEAN":
        return struct.pack("<B", 1 if value else 0)
    elif data_type == "INT8_NUMBER" or data_type == "UINT8_NUMBER":
        return struct.pack("<B", value)
    elif data_type == "INT16_DECICURRENT_VOLT":
        deci = int(round(value * 10))
        return struct.pack("<H", deci)
    elif data_type == "UTF8_STRING":
        utf = value.encode('utf-8')[:15]
        return utf + b'\x00' * (15 - len(utf))
    elif data_type == "INT16_ARRAY":
        return encode_value(value[0], "INT16_DECIDEGREE_CELSIUS") + encode_value(value[1], "INT16_DECIDEGREE_CELSIUS")
    elif data_type == "HISTORY_DATA_ARRAY":
        temps = b''.join(encode_value(t, "INT16_DECIDEGREE_CELSIUS") for t in value[:-1])
        return temps + struct.pack("<B", value[-1])
    elif data_type == "EMPTY":
        return b''
    return b''

def decode_value(bytes_val, data_type):
    if data_type == "INT16_DECIDEGREE_CELSIUS":
        val = struct.unpack("<h", bytes_val)[0]
        return val / 10.0
    elif data_type == "INT8_BOOLEAN":
        return bool(bytes_val[0])
    elif data_type == "INT8_NUMBER" or data_type == "UINT8_NUMBER":
        return bytes_val[0]
    elif data_type == "INT16_DECICURRENT_VOLT":
        val = struct.unpack("<H", bytes_val)[0]
        return val / 10.0
    elif data_type == "UTF8_STRING":
        end = bytes_val.find(b'\x00')
        return bytes_val[:end if end >= 0 else 15].decode('utf-8', errors='replace')
    elif data_type == "INT16_ARRAY":
        min_v = decode_value(bytes_val[0:2], "INT16_DECIDEGREE_CELSIUS")
        max_v = decode_value(bytes_val[2:4], "INT16_DECIDEGREE_CELSIUS")
        return [min_v, max_v]
    elif data_type == "HISTORY_DATA_ARRAY":
        temps = [decode_value(bytes_val[i:i+2], "INT16_DECIDEGREE_CELSIUS") for i in range(0, 14, 2)]
        ts = bytes_val[14]
        return temps + [ts]
    elif data_type == "EMPTY":
        return None
    return None

def get_english_desc(topic_key, value):
    if value == NO_VALUE:
        return f"{TOPICS[topic_key]['desc']} is unavailable"
    if TOPICS[topic_key]['type'] == "INT16_DECIDEGREE_CELSIUS":
        return f"{TOPICS[topic_key]['desc']} is {value}°C"
    elif TOPICS[topic_key]['type'] == "INT8_BOOLEAN":
        return f"{TOPICS[topic_key]['desc']} is {'on' if value else 'off'}" if "power" in TOPICS[topic_key]['desc'].lower() else f"{TOPICS[topic_key]['desc']} is {'active' if value else 'inactive'}"
    elif TOPICS[topic_key]['type'] == "UINT8_NUMBER" and topic_key == "BATTERY_PROTECTION_LEVEL":
        return f"{TOPICS[topic_key]['desc']} is {BATTERY_LEVELS.get(value, value)}"
    elif TOPICS[topic_key]['type'] == "INT8_NUMBER" and topic_key == "POWER_SOURCE":
        return f"{TOPICS[topic_key]['desc']} is {POWER_SOURCES.get(value, value)}"
    elif TOPICS[topic_key]['type'] == "UTF8_STRING":
        return f"{TOPICS[topic_key]['desc']} is {value}"
    elif TOPICS[topic_key]['type'] == "INT16_ARRAY":
        return f"{TOPICS[topic_key]['desc']} is {value[0]} to {value[1]}°C"
    elif TOPICS[topic_key]['type'] == "HISTORY_DATA_ARRAY":
        temps = ', '.join(f"{t}°C" for t in value[:-1])
        return f"{TOPICS[topic_key]['desc']}: temps [{temps}], timestamp {value[-1]}"
    elif TOPICS[topic_key]['type'] == "INT16_DECICURRENT_VOLT":
        return f"{TOPICS[topic_key]['desc']} is {value}V"
    elif TOPICS[topic_key]['type'] == "INT8_NUMBER":
        return f"{TOPICS[topic_key]['desc']} is {value}"
    return f"{TOPICS[topic_key]['desc']}: {value}"

async def notification_handler(sender, data: bytearray):
    action = data[0]
    payload = data[1:]
    print(f"Received: {ACTION_NAMES.get(action, 'Unknown')} {data.hex()}")
    if action == ACTION_PUB:
        topic_tuple = tuple(payload[0:4])
        topic_key = next((k for k, v in TOPICS.items() if tuple(v["param"]) == topic_tuple), None)
        if topic_key:
            value = decode_value(payload[4:], TOPICS[topic_key]["type"])
            print(get_english_desc(topic_key, value))
        else:
            print("Unknown topic")
    elif action == ACTION_ACK:
        print("ACK received")
    elif action == ACTION_NAK:
        print("NAK received")
    elif action == ACTION_PING:
        print("PING received - sending ACK")
    if action in [ACTION_PUB, ACTION_PING, ACTION_SUB, ACTION_NOP, ACTION_HELLO]:
        await client.write_gatt_char(WRITE_UUID, bytearray([ACTION_ACK]))

def get_product_type(manufacturer_data):
    if 0xFFFF in manufacturer_data:
        hex_data = ''.join(f"{b:02x}" for b in manufacturer_data[0xFFFF])
        product_hex = hex_data[4:6]
        try:
            return int(product_hex, 16)
        except:
            return 1
    return 1

async def send_packet(client, action, topic_key=None, value=None):
    packet = bytearray([action])
    if topic_key:
        packet += bytearray(TOPICS[topic_key]["param"])
        if value is not None:
            packet += encode_value(value, TOPICS[topic_key]["type"])
    await client.write_gatt_char(WRITE_UUID, packet)
    print(f"Sent: {ACTION_NAMES.get(action, 'Unknown')} {packet.hex()}")

async def set_cooler_power(client, on: bool):
    await send_packet(client, ACTION_PUB, "COOLER_POWER", on)

async def set_compartment_power(client, comp: int, on: bool):
    key = f"COMPARTMENT_{comp}_POWER"
    await send_packet(client, ACTION_PUB, key, on)

async def set_icemaker_power(client, on: bool):
    await send_packet(client, ACTION_PUB, "ICEMAKER_POWER", on)

async def set_temperature(client, comp: int, temp: float):
    key = f"COMPARTMENT_{comp}_SET_TEMPERATURE"
    await send_packet(client, ACTION_PUB, key, temp)

async def set_battery_protection(client, level: int):
    await send_packet(client, ACTION_PUB, "BATTERY_PROTECTION_LEVEL", level)

async def set_wifi_mode(client, on: bool):
    await send_packet(client, ACTION_PUB, "WIFI_MODE", on)

async def set_bluetooth_mode(client, on: bool):
    await send_packet(client, ACTION_PUB, "BLUETOOTH_MODE", on)

class SimpleQueue:
    def __init__(self):
        self.queue = []
        self.current_task = None

    def enqueue(self, packet, callback=None):
        self.queue.append((packet, callback))
        if not self.current_task:
            asyncio.create_task(self.process_queue())

    async def process_queue(self):
        while self.queue:
            packet, callback = self.queue.pop(0)
            self.current_task = asyncio.create_task(self.send_with_timeout(packet, callback))
            await self.current_task
            self.current_task = None

    async def send_with_timeout(self, packet, callback):
        await client.write_gatt_char(WRITE_UUID, packet)
        try:
            await asyncio.wait_for(asyncio.sleep(6), timeout=6)  # Wait for ACK (in practice, handle via notify)
            if callback:
                callback()
        except asyncio.TimeoutError:
            print("Timeout - retry or fail")

queue = SimpleQueue()

async def connection_watchdog(address):
    """Checks the connection status every few seconds and handles re-connection."""
    global client
    
    while True:
        await asyncio.sleep(10) # Check every 10 seconds
        
        if client and not client.is_connected:
            print(f"\n[WATCHDOG] Connection to {address} lost. Attempting reconnect...")
            try:
                # Recreate the client and attempt to connect/re-subscribe
                client = BleakClient(address)
                await client.connect()
                
                # Re-do the necessary service discovery and notifications
                await client.discover_services() # Using discover_services for 1.1.1 compatibility
                await client.start_notify(NOTIFY_UUID, notification_handler)
                
                print("[WATCHDOG] Reconnected and re-subscribed.")
                
                # Optionally, re-ping and re-subscribe to data topics
                await send_packet(client, ACTION_PING)
                
            except Exception as e:
                print(f"[WATCHDOG] Reconnect failed: {e}. Retrying...")

# Insert this function before async def main
async def connection_watchdog(address, product_type):
    """Checks the connection status every few seconds and handles re-connection."""
    global client
    
    while True:
        await asyncio.sleep(10) # Check every 10 seconds
        
        if client and not client.is_connected:
            print(f"\n[WATCHDOG] Connection to {address} lost. Attempting reconnect...")
            try:
                # Recreate the client and attempt to connect/re-subscribe
                client = BleakClient(address)
                await client.connect()
                
                # Introduce a small delay to prevent the CCCD race condition
                await asyncio.sleep(0.1) 
                
                # Re-do the necessary notification setup
                await client.start_notify(NOTIFY_UUID, notification_handler)
                
                print("[WATCHDOG] Reconnected and re-subscribed.")
                
                # Optionally, re-ping and re-subscribe to data topics
                await send_packet(client, ACTION_PING)
                
            except Exception as e:
                print(f"[WATCHDOG] Reconnect failed: {e}. Retrying...")

# Replace your existing main function entirely with this one
async def main(mac=None):
    # Discovery remains the same (returning a list of BLEDevice objects)
    print("Starting BLE device scan...")
    devices = await BleakScanner.discover()
    cfx_devices = []
    
    # Loop over the list of BLEDevice objects
    for d in devices:
        adv = getattr(d, 'advertisement', None)
        man_data = adv.manufacturer_data if adv else {}
        product = get_product_type(man_data)
        
        # Determine the device name safely
        device_name = adv.local_name if adv and adv.local_name else d.name
        
        # Filter 1: Check if the device is a known CFX product type
        if product in [1, 2, 3]:
            
            # Filter 2: Check for the required name prefix "CFX3_"
            if device_name is None or not str(device_name).startswith("CFX3_"):
                continue

            # Safely get RSSI from the advertisement data, or default to a very low value for sorting
            rssi = getattr(adv, 'rssi', -1000)
            
            # Storing the device object, product type, and the safely retrieved RSSI
            cfx_devices.append((d, product, rssi)) 
            
            # Use the retrieved RSSI in the print statement
            print(f"Found candidate: {device_name} ({d.address}), RSSI {rssi}, type {product}")

    if not cfx_devices:
        print("No CFX3_ prefixed devices found")
        return

    # Sort by the third element (RSSI) to prioritize the strongest signal
    cfx_devices.sort(key=lambda x: x[2], reverse=True)
    
    # Select the address and product type of the device with the strongest signal
    best_device, product_type, rssi = cfx_devices[0]
    address = best_device.address
    print(f"Selected best device: {best_device.name or address} (RSSI: {rssi})")

    global client
    client = BleakClient(address)
    
    # Connection retry logic for robustness
    connected = False
    for i in range(3):
        try:
            print(f"Attempting to connect to {address} (Attempt {i+1}/3)...")
            await client.connect()
            
            # --- NEW FIX: Small delay to prevent the CCCD race condition ---
            await asyncio.sleep(0.1) 
            
            connected = True
            break
        except Exception as e:
            print(f"Connection attempt {i+1} failed: {e}. Retrying...")
            if i < 2:
                await asyncio.sleep(2) # Wait before retrying
    
    if not connected:
        print("Failed to connect after multiple attempts.")
        return

    print("Connected and ready for notifications/control.")
    
    # Start notifications *after* successful connection
    await client.start_notify(NOTIFY_UUID, notification_handler)

    # 1. Start the Watchdog as a background task
    watchdog_task = asyncio.create_task(connection_watchdog(address, product_type))

    # 2. Send your initial commands
    await send_packet(client, ACTION_PING)
    await asyncio.sleep(1)

    sub_key = {1: "SUBSCRIBE_APP_SZ", 2: "SUBSCRIBE_APP_SZI", 3: "SUBSCRIBE_APP_DZ"}[product_type]
    await send_packet(client, ACTION_SUB, sub_key)

    # Example sets
    await set_cooler_power(client, True)
    await set_temperature(client, 0, 4.0)
    await set_battery_protection(client, 1)

    # 3. Use an endless task to keep the main thread alive, instead of a simple sleep
    print("\n--- Running indefinitely. Press Ctrl+C to stop. ---\n")
    await asyncio.Future()
    
asyncio.run(main())