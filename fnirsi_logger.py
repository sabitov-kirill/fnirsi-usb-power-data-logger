#!/usr/bin/env python3

import logging
import sys
import os.path
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Callable

import usb.core
import usb.util
import argparse

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class DeviceModel(Enum):
    FNB48 = auto()
    C1 = auto()
    FNB58 = auto()
    FNB48S = auto()

@dataclass
class DeviceInfo:
    vid: int
    pid: int
    model: DeviceModel
    refresh_rate: float

DEVICE_MAP = {
    # FNB48
    # Bus 001 Device 020: ID 0483:003a STMicroelectronics FNB-48
    (0x0483, 0x003A): DeviceInfo(0x0483, 0x003A, DeviceModel.FNB48, 0.003),
    # C1
    # Bus 001 Device 029: ID 0483:003b STMicroelectronics USB Tester
    (0x0483, 0x003B): DeviceInfo(0x0483, 0x003B, DeviceModel.C1, 0.003),
    # FNB58
    (0x2E3C, 0x5558): DeviceInfo(0x2E3C, 0x5558, DeviceModel.FNB58, 1.0),
    # FNB48S
    # Bus 001 Device 003: ID 2e3c:0049 FNIRSI USB Tester
    (0x2E3C, 0x0049): DeviceInfo(0x2E3C, 0x0049, DeviceModel.FNB48S, 1.0),
}

@dataclass
class MeasurementData:
    timestamp: float
    voltage: float
    current: float
    dp: float
    dn: float
    temperature: float
    energy: float
    capacity: float

class USBMeter:
    def __init__(self, verbose: bool = False, crc: bool = False, alpha: float = 0.9):
        self.verbose = verbose
        self.use_crc = crc
        self.alpha = alpha
        self.energy = 0.0
        self.capacity = 0.0
        self.temp_ema = None
        self.crc_calculator = self._setup_crc() if crc else None
        self.device_info = None
        self.device = None
        self.ep_in = None
        self.ep_out = None

    def _setup_crc(self) -> Optional[Callable]:
        try:
            import crc
            width = 8
            poly = 0x39
            init_value = 0x42
            final_xor_value = 0x00
            config = crc.Configuration(
                width, poly, init_value, final_xor_value, 
                reverse_input=False, reverse_output=False
            )
            if hasattr(crc, "CrcCalculator"):
                return crc.CrcCalculator(config, use_table=True).calculate_checksum
            return crc.Calculator(config, optimized=True).checksum
        except ImportError:
            logger.warning("CRC package not found, disabling CRC checks")
            return None

    def find_device(self) -> None:
        for (vid, pid), info in DEVICE_MAP.items():
            device = usb.core.find(idVendor=vid, idProduct=pid)
            if device:
                self.device = device
                self.device_info = info
                logger.debug(f"Found {info.model.name} device")
                return
        raise RuntimeError("No supported USB meter found")

    def setup_device(self) -> None:
        self.device.reset()
        
        if self.verbose:
            self._print_device_info()

        # Find and setup HID interface
        interface_num = self._find_hid_interface()
        self._detach_kernel_driver(interface_num)
        
        # Configure device
        self.device.set_configuration()
        cfg = self.device.get_active_configuration()
        intf = cfg[(interface_num, 0)]

        # Get endpoints
        self.ep_out = self._find_endpoint(intf, usb.util.ENDPOINT_OUT)
        self.ep_in = self._find_endpoint(intf, usb.util.ENDPOINT_IN)

    def _find_hid_interface(self) -> int:
        for cfg in self.device:
            for interface in cfg:
                if interface.bInterfaceClass == 0x03:  # HID class
                    return interface.bInterfaceNumber
        raise RuntimeError("No HID interface found")

    def _detach_kernel_driver(self, interface_num: int) -> None:
        if self.device.is_kernel_driver_active(interface_num):
            try:
                self.device.detach_kernel_driver(interface_num)
            except usb.core.USBError as e:
                raise RuntimeError(f"Could not detach kernel driver: {e}")

    def _find_endpoint(self, interface, direction) -> usb.core.Endpoint:
        return usb.util.find_descriptor(
            interface,
            custom_match=lambda e: 
                usb.util.endpoint_direction(e.bEndpointAddress) == direction
        )

    def _print_device_info(self) -> None:
        logger.debug("Device configuration:")
        for cfg in self.device:
            logger.debug(f"Config {cfg.bConfigurationValue}")
            for interface in cfg:
                logger.debug(f"  Interface {interface.bInterfaceNumber}")
                for ep in interface:
                    logger.debug(f"    Endpoint {ep.bEndpointAddress:02x}")

    def initialize_communication(self) -> None:
        init_sequence = [
            (b"\xaa\x81", b"\x8e"),
            (b"\xaa\x82", b"\x96"),
        ]
        
        if self.device_info.model in (DeviceModel.FNB58, DeviceModel.FNB48S):
            init_sequence.append((b"\xaa\x82", b"\x96"))
        else:
            init_sequence.append((b"\xaa\x83", b"\x9e"))

        for prefix, suffix in init_sequence:
            self.ep_out.write(prefix + b"\x00" * 61 + suffix)
            time.sleep(0.01)

    def decode_packet(self, data: bytes, timestamp: float) -> Optional[MeasurementData]:
        # Data is 64 bytes (64 bytes of HID data minus vendor constant 0xaa)
        # First byte is HID vendor constant 0xaa
        # Second byte is payload type:
        #    0x04 is data packet
        #    Other types (0x03 and maybe other ones) is unknown
        # Next 4 samples each 15 bytes. 60 bytes total.
        # At the end 2 bytes:
        #   1 byte is semi constant with unknown purpose.
        #   1 byte (last) is a 8-bit CRC checksum

        if data[1] != 0x04:  # Not a data packet
            return None

        if self.use_crc and self.crc_calculator:
            if not self._verify_crc(data):
                return None

        measurements = []
        base_time = timestamp - 0.04  # 4 samples, 10ms each
        
        for i in range(4):
            offset = 2 + 15 * i
            measurement = self._decode_measurement(data[offset:offset+15], base_time + i * 0.01)
            measurements.append(measurement)

        return measurements[-1]  # Return most recent measurement

    def _decode_measurement(self, data: bytes, timestamp: float) -> MeasurementData:
        voltage = int.from_bytes(data[0:4], 'little') / 100000
        current = int.from_bytes(data[4:8], 'little') / 100000
        dp = int.from_bytes(data[8:10], 'little') / 1000
        dn = int.from_bytes(data[10:12], 'little') / 1000
        temp_C = int.from_bytes(data[13:15], 'little') / 10.0

        # Update running totals
        power = voltage * current
        self.energy += power * 0.01  # 10ms interval
        self.capacity += current * 0.01

        # Update EMA temperature
        if self.temp_ema is None:
            self.temp_ema = temp_C
        else:
            self.temp_ema = temp_C * (1.0 - self.alpha) + self.temp_ema * self.alpha

        return MeasurementData(
            timestamp=timestamp,
            voltage=voltage,
            current=current,
            dp=dp,
            dn=dn,
            temperature=self.temp_ema,
            energy=self.energy,
            capacity=self.capacity
        )

    def _verify_crc(self, data: bytes) -> bool:
        actual = data[-1]
        expected = self.crc_calculator(bytearray(data[1:-1]))
        if actual != expected:
            logger.warning(
                f"CRC mismatch: expected {expected:02x}, got {actual:02x}"
            )
            return False
        return True

    def run(self) -> None:
        print("timestamp voltage_V current_A dp_V dn_V temp_C_ema energy_Ws capacity_As")
        
        next_refresh = time.time() + self.device_info.refresh_rate
        stop = False

        while not stop:
            try:
                data = self.ep_in.read(64, timeout=5000)
                measurement = self.decode_packet(data, time.time())
                
                if measurement:
                    print(
                        f"{measurement.timestamp:.3f} {measurement.voltage:7.5f} "
                        f"{measurement.current:7.5f} {measurement.dp:5.3f} "
                        f"{measurement.dn:5.3f} {measurement.temperature:6.3f} "
                        f"{measurement.energy:.6f} {measurement.capacity:.6f}"
                    )

                if time.time() >= next_refresh:
                    next_refresh = time.time() + self.device_info.refresh_rate
                    self.ep_out.write(b"\xaa\x83" + b"\x00" * 61 + b"\x9e")

                if os.path.exists("fnirsi_stop"):
                    stop = True

            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received, stopping...")
                stop = True

        self._drain_buffer()

    def _drain_buffer(self) -> None:
        logger.debug("Draining USB buffer...")
        try:
            while True:
                data = self.ep_in.read(64, timeout=1000)
                if data and self.verbose:
                    logger.debug(f"Drained {len(data)} bytes")
        except usb.core.USBTimeoutError:
            logger.debug("Buffer drain complete")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--crc", type=bool, default=False, help="Enable CRC checks")
    parser.add_argument("--verbose", type=bool, default=False, help="Enable verbose logging")
    parser.add_argument("--alpha", type=float, default=0.9, help="Temperature EMA factor")
    args = parser.parse_args()

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    meter = USBMeter(verbose=args.verbose, crc=args.crc, alpha=args.alpha)
    
    try:
        meter.find_device()
        meter.setup_device()
        meter.initialize_communication()
        meter.run()
    except Exception as e:
        logger.error(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()