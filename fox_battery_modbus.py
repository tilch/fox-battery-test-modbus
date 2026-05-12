#!/usr/bin/env python3
"""
Modbus TCP client for FOX ESS G-MAX: telemetry (input registers, FC 0x04) and optional
controlled writes for active power (holding registers, FC 0x06).

Register map follows the vendor workbook
`G-MAX Communication Protocol modbusTcp-EN-20250805.xlsx` (EMS-Telemetry, BMS-Telemetry,
EMS-Remote Adjustment for power command).
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from dotenv import load_dotenv
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

_PROJECT_DIR = Path(__file__).resolve().parent
load_dotenv(_PROJECT_DIR / ".env")


def _default_modbus_host() -> str:
    return os.getenv("FOX_MODBUS_HOST", "10.97.29.49").strip() or "10.97.29.49"


def _default_modbus_port() -> int:
    raw = os.getenv("FOX_MODBUS_PORT", "502").strip() or "502"
    try:
        return int(raw, 10)
    except ValueError as e:
        raise SystemExit(f"Invalid FOX_MODBUS_PORT in environment: {raw!r}") from e


@dataclass(frozen=True)
class Reg:
    """Single telemetry field."""

    key: str
    address: int
    dtype: str  # int16 | uint16 | uint32
    accuracy: float
    offset: float
    unit: str
    section: str
    description: str = ""


@dataclass(frozen=True)
class HoldingReg:
    """Single holding-register status field."""

    key: str
    address: int
    dtype: str  # int16 | uint16
    accuracy: float
    offset: float
    unit: str
    section: str
    description: str = ""


def _i16(raw: int) -> int:
    return struct.unpack(">h", struct.pack(">H", raw & 0xFFFF))[0]


def _decode_uint32_pair(word0: int, word1: int) -> int:
    """Decode two consecutive input registers as a 32-bit value.

    FOX firmware mixes patterns observed in the field:
    - ``[313, 0]`` style: value lives in the first word only (cumulative kWh, etc.).
    - ``[0, 53036]`` style: value lives in the second word only (heartbeat).
    - If both words are non-zero, assume little-endian word order (low word first).
    """
    a = int(word0) & 0xFFFF
    b = int(word1) & 0xFFFF
    if b == 0:
        return a
    if a == 0:
        return b
    return a | (b << 16)


def scale(raw: float, offset: float, accuracy: float) -> float:
    if accuracy == 0:
        return float(raw) + offset
    return (raw + offset) / accuracy


# EMS-Remote Adjustment (holding registers, FC 0x03 read / 0x06 write per workbook).
HOLDING_INSTRUCTION_MODE = 102  # 2 = total instructions (use register 105 for power).
HOLDING_CONTROL_SOURCE = 104  # 1 = remote (per workbook).
HOLDING_TOTAL_ACTIVE_POWER_KW = 105  # INT16, signed kW * 10: charge negative, discharge positive.
ACTIVE_POWER_SCALE = 10
INT16_MIN = -32768
INT16_MAX = 32767


def encode_total_active_power_kw(signed_kw: float) -> int:
    """Scale signed kW to INT16 raw (workbook accuracy 10); return Modbus uint16 wire value."""
    raw = int(round(float(signed_kw) * ACTIVE_POWER_SCALE))
    if raw < INT16_MIN or raw > INT16_MAX:
        raise ValueError(f"active power raw {raw} out of INT16 range for {signed_kw} kW")
    return raw & 0xFFFF


def write_holding_u16(client: ModbusTcpClient, address: int, value_u16: int, *, device_id: int) -> None:
    if value_u16 < 0 or value_u16 > 0xFFFF:
        raise ValueError(f"holding value out of range: {value_u16}")
    try:
        rr = client.write_register(address=address, value=value_u16, device_id=device_id)
    except ModbusException as e:
        raise RuntimeError(f"Modbus exception writing holding register @{address}: {e}") from e
    if rr.isError():
        raise RuntimeError(f"Modbus error writing holding register @{address}: {rr}")


def _prepare_remote_total_power_mode(
    client: ModbusTcpClient,
    *,
    device_id: int,
    address_offset: int,
) -> None:
    """Best-effort: total-instruction mode + remote control source (may be required before 105 takes effect)."""
    write_holding_u16(
        client,
        HOLDING_INSTRUCTION_MODE + address_offset,
        2,
        device_id=device_id,
    )
    write_holding_u16(
        client,
        HOLDING_CONTROL_SOURCE + address_offset,
        1,
        device_id=device_id,
    )


def write_total_active_power_kw(
    client: ModbusTcpClient,
    signed_kw: float,
    *,
    device_id: int = 1,
    address_offset: int = 0,
) -> int:
    """Write holding register 105 with signed active power (kW). Returns wire uint16 written."""
    wire = encode_total_active_power_kw(signed_kw)
    write_holding_u16(
        client,
        HOLDING_TOTAL_ACTIVE_POWER_KW + address_offset,
        wire,
        device_id=device_id,
    )
    return wire


def charge_with_power_kw(
    client: ModbusTcpClient,
    power_kw: float,
    *,
    device_id: int = 1,
    address_offset: int = 0,
    max_abs_kw: float = 5.0,
    prepare: bool = True,
) -> float:
    """Request charge using a positive magnitude (kW); writes negative power to register 105."""
    if power_kw <= 0:
        raise ValueError("power_kw must be positive (magnitude)")
    mag = min(float(power_kw), float(max_abs_kw))
    signed = -mag
    if prepare:
        _prepare_remote_total_power_mode(client, device_id=device_id, address_offset=address_offset)
    write_total_active_power_kw(client, signed, device_id=device_id, address_offset=address_offset)
    return signed


def discharge_with_power_kw(
    client: ModbusTcpClient,
    power_kw: float,
    *,
    device_id: int = 1,
    address_offset: int = 0,
    max_abs_kw: float = 5.0,
    prepare: bool = True,
) -> float:
    """Request discharge using a positive magnitude (kW); writes positive power to register 105."""
    if power_kw <= 0:
        raise ValueError("power_kw must be positive (magnitude)")
    mag = min(float(power_kw), float(max_abs_kw))
    signed = mag
    if prepare:
        _prepare_remote_total_power_mode(client, device_id=device_id, address_offset=address_offset)
    write_total_active_power_kw(client, signed, device_id=device_id, address_offset=address_offset)
    return signed


# Workbook addresses (Protocol "address" column). Default --address-offset 0 uses these as PDU start addresses.
# If reads fail with Illegal Data Address, try --address-offset -1 (some stacks use 0-based docs).
REGISTERS: tuple[Reg, ...] = (
    # EMS-Telemetry (summary)
    Reg("ems_pcs_active_power_kw", 101, "int16", 10, 0, "kW", "EMS", "PCS actual total active power (summary)"),
    Reg("ems_pcs_reactive_power_kvar", 102, "int16", 10, 0, "kVar", "EMS", "PCS actual total reactive power (summary)"),
    Reg("ems_total_voltage_v", 103, "int16", 10, 0, "V", "EMS", "Total voltage (summary)"),
    Reg("ems_total_current_a", 104, "int16", 10, 0, "A", "EMS", "Total current (summary)"),
    Reg("ems_total_soc_pct", 105, "uint16", 1, 0, "%", "EMS", "Total SOC (summary)"),
    Reg("ems_battery_status", 106, "uint16", 1, 0, "", "EMS", "0:Fault 1:Stop 2:Run 3:Charge 4:Discharge"),
    Reg("ems_battery_count", 107, "uint16", 1, 0, "", "EMS", "Total number of batteries (summary)"),
    Reg("ems_avg_battery_temp_c", 108, "int16", 1, 0, "°C", "EMS", "Average battery temperature (summary)"),
    Reg("ems_max_battery_temp_diff_c", 109, "int16", 1, 0, "°C", "EMS", "Maximum battery temperature difference (summary)"),
    Reg("ems_max_cell_voltage_v", 110, "uint16", 1000, 0, "V", "EMS", "Maximum single cell voltage (summary)"),
    Reg("ems_min_cell_voltage_v", 111, "uint16", 1000, 0, "V", "EMS", "Minimum single cell voltage (summary)"),
    Reg("ems_system_status", 114, "uint16", 1, 0, "", "EMS", "0:Stop 1:Standby 2:Run 3:Fault"),
    Reg("ems_heartbeat", 136, "uint32", 1, 0, "", "EMS", "Heartbeat (UINT32)"),
    # BMS-Telemetry (stack / array)
    Reg("bms_stack_soc_pct", 200, "uint16", 1, 0, "%", "BMS-stack", "Stack SOC"),
    Reg("bms_stack_soh_pct", 201, "uint16", 1, 0, "%", "BMS-stack", "Stack SOH"),
    Reg("bms_stack_status", 202, "uint16", 1, 0, "", "BMS-stack", "0:Stop 1:Charge 2:Discharge 3:Standby 4:Fault"),
    Reg("bms_stack_voltage_v", 203, "int16", 10, 0, "V", "BMS-stack", "Stack voltage"),
    Reg("bms_stack_current_a", 204, "int16", 10, 0, "A", "BMS-stack", "Stack current"),
    Reg("bms_stack_max_cell_voltage_v", 205, "int16", 1000, 0, "V", "BMS-stack", "Maximum single cell voltage"),
    Reg("bms_stack_min_cell_voltage_v", 206, "int16", 1000, 0, "V", "BMS-stack", "Minimum single cell voltage"),
    Reg("bms_stack_max_cell_temp_c", 207, "int16", 1, 0, "°C", "BMS-stack", "Maximum monomer temperature"),
    Reg("bms_stack_min_cell_temp_c", 208, "int16", 1, 0, "°C", "BMS-stack", "Minimum monomer temperature"),
    Reg("bms_stack_max_cell_soc_pct", 209, "uint16", 1, 0, "%", "BMS-stack", "Maximum single cell SOC"),
    Reg("bms_stack_min_cell_soc_pct", 210, "uint16", 1, 0, "%", "BMS-stack", "Minimum single cell SOC"),
    Reg("bms_stack_max_cell_pressure_diff_mv", 223, "uint16", 1, 0, "mV", "BMS-stack", "Maximum pressure difference of monomer"),
    Reg("bms_stack_power_kw", 224, "int16", 1, 0, "kW", "BMS-stack", "Stack power"),
    Reg("bms_cumulative_charge_kwh", 227, "uint32", 1, 0, "kWh", "BMS-stack", "Cumulative cluster charging capacity"),
    Reg("bms_cumulative_discharge_kwh", 229, "uint32", 1, 0, "kWh", "BMS-stack", "Cumulative cluster discharge capacity"),
    Reg("bms_rechargeable_capacity_kwh", 231, "uint32", 10, 0, "kWh", "BMS-stack", "Rechargeable capacity"),
    Reg("bms_dischargeable_capacity_kwh", 233, "uint32", 10, 0, "kWh", "BMS-stack", "Dischargeable capacity"),
    # BMS-Telemetry (cluster_0) — PDF example: cluster SOC at point/address 2200
    Reg("bms_cluster0_soc_pct", 2200, "uint16", 1, 0, "%", "BMS-cluster0", "Cluster SOC"),
    Reg("bms_cluster0_soh_pct", 2201, "uint16", 1, 0, "%", "BMS-cluster0", "Cluster SOH"),
    Reg("bms_cluster0_status", 2202, "uint16", 1, 0, "", "BMS-cluster0", "0:Stop 1:Charging 2:Discharging 3:Standby 4:Fault"),
    Reg("bms_cluster0_voltage_v", 2203, "int16", 10, 0, "V", "BMS-cluster0", "Cluster voltage"),
    Reg("bms_cluster0_current_a", 2204, "int16", 10, 0, "A", "BMS-cluster0", "Cluster current"),
    Reg("bms_cluster0_max_cell_voltage_v", 2205, "int16", 1000, 0, "V", "BMS-cluster0", "Maximum single cell voltage"),
    Reg("bms_cluster0_min_cell_voltage_v", 2206, "int16", 1000, 0, "V", "BMS-cluster0", "Minimum single cell voltage"),
    Reg("bms_cluster0_max_cell_temp_c", 2207, "int16", 1, 0, "°C", "BMS-cluster0", "Maximum monomer temperature"),
    Reg("bms_cluster0_min_cell_temp_c", 2208, "int16", 1, 0, "°C", "BMS-cluster0", "Minimum monomer temperature"),
    Reg("bms_cluster0_max_cell_soc_pct", 2209, "uint16", 1, 0, "%", "BMS-cluster0", "Maximum single cell SOC"),
    Reg("bms_cluster0_min_cell_soc_pct", 2210, "uint16", 1, 0, "%", "BMS-cluster0", "Minimum single cell SOC"),
)

CONTROL_REGISTERS: tuple[HoldingReg, ...] = (
    HoldingReg(
        "ems_control_source",
        HOLDING_CONTROL_SOURCE,
        "uint16",
        1,
        0,
        "",
        "EMS-control",
        "Control source: 0=local, 1=remote, other=vendor-specific",
    ),
)


def _addresses_used(regs: Iterable[Reg]) -> list[int]:
    addrs: list[int] = []
    for r in regs:
        if r.dtype == "uint32":
            addrs.extend([r.address, r.address + 1])
        else:
            addrs.append(r.address)
    return sorted(set(addrs))


def _merge_read_ranges(sorted_addrs: list[int], max_count: int = 120) -> list[tuple[int, int]]:
    """Return (start, count) chunks for read_input_registers.

    Only strictly consecutive addresses in ``sorted_addrs`` are merged, so we never
    read "holes" between documented registers (avoids illegal address errors).
    """
    if not sorted_addrs:
        return []
    ranges: list[tuple[int, int]] = []
    start = sorted_addrs[0]
    prev = sorted_addrs[0]
    for a in sorted_addrs[1:]:
        if a == prev + 1 and (a - start + 1) <= max_count:
            prev = a
            continue
        ranges.append((start, prev - start + 1))
        start = prev = a
    ranges.append((start, prev - start + 1))
    return ranges


def _read_input_map(
    client: ModbusTcpClient,
    unit: int,
    address_offset: int,
    ranges: list[tuple[int, int]],
) -> dict[int, int]:
    addr_map: dict[int, int] = {}
    for start, count in ranges:
        pdu_start = start + address_offset
        try:
            rr = client.read_input_registers(address=pdu_start, count=count, device_id=unit)
        except ModbusException as e:
            raise RuntimeError(f"Modbus exception reading input registers @{pdu_start} count={count}: {e}") from e
        if rr.isError():
            raise RuntimeError(f"Modbus error reading input registers @{pdu_start} count={count}: {rr}")
        for i, word in enumerate(rr.registers):
            addr_map[start + i] = int(word) & 0xFFFF
    return addr_map


def _read_holding_map(
    client: ModbusTcpClient,
    unit: int,
    address_offset: int,
    regs: Iterable[HoldingReg],
) -> dict[int, int]:
    addr_map: dict[int, int] = {}
    for r in regs:
        address = r.address + address_offset
        try:
            rr = client.read_holding_registers(address=address, count=1, device_id=unit)
        except ModbusException as e:
            raise RuntimeError(f"Modbus exception reading holding register @{address}: {e}") from e
        if rr.isError():
            raise RuntimeError(f"Modbus error reading holding register @{address}: {rr}")
        addr_map[r.address] = int(rr.registers[0]) & 0xFFFF
    return addr_map


def _decode_regs(addr_map: dict[int, int], regs: Iterable[Reg]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for r in regs:
        if r.dtype == "uint16":
            raw_u = addr_map.get(r.address)
            if raw_u is None:
                out[r.key] = None
                continue
            if raw_u == 0xFFFF and r.key in {
                "bms_stack_max_cell_soc_pct",
                "bms_stack_min_cell_soc_pct",
                "bms_cluster0_max_cell_soc_pct",
                "bms_cluster0_min_cell_soc_pct",
            }:
                out[r.key] = None
                continue
            out[r.key] = scale(float(raw_u), r.offset, r.accuracy)
        elif r.dtype == "int16":
            raw_u = addr_map.get(r.address)
            if raw_u is None:
                out[r.key] = None
                continue
            raw_i = float(_i16(raw_u))
            out[r.key] = scale(raw_i, r.offset, r.accuracy)
        elif r.dtype == "uint32":
            w0 = addr_map.get(r.address)
            w1 = addr_map.get(r.address + 1)
            if w0 is None or w1 is None:
                out[r.key] = None
                continue
            raw_u32 = float(_decode_uint32_pair(w0, w1))
            out[r.key] = scale(raw_u32, r.offset, r.accuracy)
        else:
            raise ValueError(f"Unknown dtype {r.dtype}")
    return out


def _decode_holding_regs(addr_map: dict[int, int], regs: Iterable[HoldingReg]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for r in regs:
        raw_u = addr_map.get(r.address)
        if raw_u is None:
            out[r.key] = None
            continue
        if r.dtype == "uint16":
            out[r.key] = scale(float(raw_u), r.offset, r.accuracy)
        elif r.dtype == "int16":
            out[r.key] = scale(float(_i16(raw_u)), r.offset, r.accuracy)
        else:
            raise ValueError(f"Unknown holding dtype {r.dtype}")
    return out


def _enriched_payload(host: str, port: int, unit: int, values: dict[str, Any]) -> dict[str, Any]:
    meta = {
        r.key: {"section": r.section, "measurement_unit": r.unit, "address": r.address, "description": r.description}
        for r in REGISTERS
    }
    meta.update(
        {
            r.key: {
                "section": r.section,
                "measurement_unit": r.unit,
                "address": r.address,
                "register_type": "holding",
                "description": r.description,
            }
            for r in CONTROL_REGISTERS
        }
    )
    return {
        "host": host,
        "port": port,
        "unit": unit,
        "telemetry": values,
        "fields": meta,
    }


def _run_read_telemetry(args: argparse.Namespace) -> int:
    used = _addresses_used(REGISTERS)
    ranges = _merge_read_ranges(used)

    client = ModbusTcpClient(host=args.host, port=args.port, timeout=args.timeout)
    if not client.connect():
        print(f"ERROR: could not connect to {args.host}:{args.port}", file=sys.stderr)
        return 2

    try:
        addr_map = _read_input_map(client, unit=args.unit, address_offset=args.address_offset, ranges=ranges)
        values = _decode_regs(addr_map, REGISTERS)
        control_addr_map = _read_holding_map(
            client,
            unit=args.unit,
            address_offset=args.address_offset,
            regs=CONTROL_REGISTERS,
        )
        values.update(_decode_holding_regs(control_addr_map, CONTROL_REGISTERS))
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 3
    finally:
        client.close()

    if args.format == "json":
        print(json.dumps(_enriched_payload(args.host, args.port, args.unit, values), indent=2))
        return 0

    print(f"FOX telemetry  {args.host}:{args.port}  unit={args.unit}  address_offset={args.address_offset}\n")
    current_section = ""
    for r in REGISTERS:
        if r.section != current_section:
            current_section = r.section
            print(f"[{current_section}]")
        v = values.get(r.key)
        if v is None:
            disp = "n/a"
        elif isinstance(v, float) and v == int(v):
            disp = str(int(v))
        else:
            disp = f"{v:.4f}".rstrip("0").rstrip(".")
        unit_s = f" {r.unit}" if r.unit else ""
        desc = f"  # {r.description}" if r.description else ""
        print(f"  {r.key}: {disp}{unit_s}{desc}")
    for r in CONTROL_REGISTERS:
        if r.section != current_section:
            current_section = r.section
            print(f"[{current_section}]")
        v = values.get(r.key)
        if v is None:
            disp = "n/a"
        elif isinstance(v, float) and v == int(v):
            disp = str(int(v))
        else:
            disp = f"{v:.4f}".rstrip("0").rstrip(".")
        unit_s = f" {r.unit}" if r.unit else ""
        desc = f"  # {r.description}" if r.description else ""
        print(f"  {r.key}: {disp}{unit_s}{desc}")
    return 0


def _run_power_write(args: argparse.Namespace, *, discharge: bool) -> int:
    if not args.i_understand_writes:
        print(
            "ERROR: refusing to write without --i-understand-writes (Modbus writes affect the live system).",
            file=sys.stderr,
        )
        return 4

    client = ModbusTcpClient(host=args.host, port=args.port, timeout=args.timeout)
    if not client.connect():
        print(f"ERROR: could not connect to {args.host}:{args.port}", file=sys.stderr)
        return 2

    try:
        if discharge:
            signed = discharge_with_power_kw(
                client,
                args.power_kw,
                device_id=args.unit,
                address_offset=args.address_offset,
                max_abs_kw=args.max_kw,
                prepare=not args.no_prepare,
            )
            verb = "discharge"
        else:
            signed = charge_with_power_kw(
                client,
                args.power_kw,
                device_id=args.unit,
                address_offset=args.address_offset,
                max_abs_kw=args.max_kw,
                prepare=not args.no_prepare,
            )
            verb = "charge"
    except (ValueError, RuntimeError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 3
    finally:
        client.close()

    raw_i16 = _i16(encode_total_active_power_kw(signed))
    print(
        f"OK: {verb} request  signed={signed:g} kW  raw_INT16={raw_i16}  "
        f"holding@{HOLDING_TOTAL_ACTIVE_POWER_KW + args.address_offset}  "
        f"prepare={'off' if args.no_prepare else '102=2,104=1'}"
    )
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description="FOX ESS G-MAX Modbus TCP: read telemetry (FC 0x04) and optional active-power writes (FC 0x06)."
    )
    p.add_argument(
        "--host",
        default=_default_modbus_host(),
        help="Modbus TCP host (default: FOX_MODBUS_HOST from .env, else 10.97.29.49).",
    )
    p.add_argument(
        "--port",
        type=int,
        default=_default_modbus_port(),
        help="Modbus TCP port (default: FOX_MODBUS_PORT from .env, else 502).",
    )
    p.add_argument("--unit", type=int, default=1, help="Modbus slave / unit id")
    p.add_argument("--timeout", type=float, default=5.0, help="Socket timeout (seconds)")
    p.add_argument(
        "--address-offset",
        type=int,
        default=0,
        help="Added to workbook register addresses before access (try -1 if Illegal Data Address).",
    )
    p.add_argument("--format", choices=("text", "json"), default="text", help="Output format for read (telemetry).")

    sub = p.add_subparsers(dest="cmd", required=False)

    sub.add_parser("read", help="Read input-register telemetry (default if no subcommand).")

    charge_p = sub.add_parser("charge", help="Write holding 105: charge at given kW magnitude (negative command).")
    charge_p.add_argument("power_kw", type=float, help="Positive magnitude in kW (capped by --max-kw).")
    charge_p.add_argument(
        "--max-kw",
        type=float,
        default=5.0,
        metavar="N",
        help="Absolute cap on requested kW (default: 5).",
    )
    charge_p.add_argument(
        "--no-prepare",
        action="store_true",
        help="Do not write 102=2 and 104=1 before power (use if your site is already in remote/total mode).",
    )
    charge_p.add_argument(
        "--i-understand-writes",
        action="store_true",
        help="Required acknowledgement before any Modbus write.",
    )

    discharge_p = sub.add_parser("discharge", help="Write holding 105: discharge at given kW magnitude (positive command).")
    discharge_p.add_argument("power_kw", type=float, help="Positive magnitude in kW (capped by --max-kw).")
    discharge_p.add_argument("--max-kw", type=float, default=5.0, metavar="N", help="Absolute cap on requested kW (default: 5).")
    discharge_p.add_argument(
        "--no-prepare",
        action="store_true",
        help="Do not write 102=2 and 104=1 before power (use if your site is already in remote/total mode).",
    )
    discharge_p.add_argument(
        "--i-understand-writes",
        action="store_true",
        help="Required acknowledgement before any Modbus write.",
    )

    args = p.parse_args()

    if args.cmd in (None, "read"):
        return _run_read_telemetry(args)
    if args.cmd == "charge":
        return _run_power_write(args, discharge=False)
    if args.cmd == "discharge":
        return _run_power_write(args, discharge=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
