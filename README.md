# FOX house battery — Modbus TCP telemetry

Python client for **FOX ESS G-MAX** style Modbus TCP (port **502**): **telemetry** (input registers) and optional **active power commands** (holding registers), using the register map in:

- `G-MAX Communication Protocol modbusTcp-EN-20250805.xlsx` — **EMS-Telemetry** / **BMS-Telemetry** use **FC 0x04** (read input registers). **EMS-Remote Adjustment** documents **FC 0x06** writes to holding registers (e.g. total active power at **105**).
- `Modbus TCP example (1).pdf` — short examples (e.g. cluster SOC at address **2200** on the BMS telemetry sheet).

Default Modbus target is **`FOX_MODBUS_HOST`:`FOX_MODBUS_PORT`** from a **`.env`** file next to `fox_battery_modbus.py` (see [`.env.example`](.env.example)). If `.env` is missing, it falls back to **`10.97.29.49:502`**. Unit id defaults to **1** (workbook “Slave Address”).

## What EMS, BMS stack, and BMS-cluster0 mean

The script prints **three blocks** because the FOX protocol exposes **three scopes of registers** on the **same** Modbus TCP endpoint (one IP, one unit id)—not three separate physical batteries.

| Script section | Meaning | Typical content |
|----------------|---------|-------------------|
| **EMS** | **Energy Management System** — system / PCS / controller summary | Overall power, pack voltage and current as seen at system level, total SOC, summary battery status, system run state, heartbeat, etc. Matches the workbook **EMS-Telemetry** sheet (addresses such as 101–114, 136, …). |
| **BMS stack** | **Battery Management System** at **whole-pack / array** level | Stack SOC/SOH, stack voltage and current, cell voltage and temperature extremes, cumulative energy, usable capacity, etc. Matches the **`array_…`** / general **BMS-Telemetry** rows (addresses in the **200+** range). |
| **BMS-cluster0** | **BMS for one cluster** (first cluster in the map) | Per-cluster SOC, SOH, status, voltage, current, and cell extremes for that segment. The workbook uses **`cluster_0_…`** and addresses around **2200+** (the PDF example “cluster SOC at 2200” is here). More clusters would appear as additional `cluster_1`, … ranges in the protocol; this project only reads **cluster 0** in the first pass. |

**Why three readouts?** FOX documents **EMS** (site/system view), **BMS stack** (entire pack view), and **BMS cluster** (per-segment view) as **separate Modbus tables**. The CLI groups fields that way so the output matches the vendor sheets.

## Setup

```bash
cd /Users/tilc/fox-battery-test
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Configuration (host / port)

```bash
cp .env.example .env
# Edit .env: FOX_MODBUS_HOST and FOX_MODBUS_PORT
```

CLI flags **`--host`** and **`--port`** still override `.env` when you pass them.

## Run

### Read telemetry (default)

No subcommand = same as `read`:

```bash
python fox_battery_modbus.py
python fox_battery_modbus.py read
```

JSON (includes field metadata):

```bash
python fox_battery_modbus.py --format json
```

Override connection (**global options must come before** the subcommand name):

```bash
python fox_battery_modbus.py --host 10.97.29.49 --port 502 --unit 1 --timeout 5 read
python fox_battery_modbus.py --timeout 5 charge 0.2 --max-kw 2 --i-understand-writes
```

If you get **Illegal Data Address** (exception 2), some gateways use a different base; try:

```bash
python fox_battery_modbus.py --address-offset -1
```

### Charge / discharge (writes)

The vendor workbook (**EMS-Remote Adjustment**) defines **holding register 105** — *Total active power command*: **INT16**, scale **×10** per kW. Convention: **negative kW = charge**, **positive kW = discharge** (same as `Modbus TCP example (1).pdf`).

Python API (after you connect a `ModbusTcpClient`):

- `charge_with_power_kw(client, power_kw, ...)` — `power_kw` is a **positive magnitude**; the script writes a **negative** command to **105**.
- `discharge_with_power_kw(client, power_kw, ...)` — writes a **positive** command to **105**.

Optional **prepare** step (default **on**): writes **102 = 2** (total instructions) and **104 = 1** (remote control source) before **105**, which the workbook implies is often needed for Modbus power to take effect. The EMS must still allow **remote** operation (often enabled in the local EMS UI first; see PDF).

CLI (writes are blocked unless you opt in):

```bash
python fox_battery_modbus.py charge 0.5 --i-understand-writes
python fox_battery_modbus.py discharge 0.5 --i-understand-writes
python fox_battery_modbus.py --host 10.97.29.49 --timeout 5 charge 0.2 --max-kw 2 --i-understand-writes
python fox_battery_modbus.py charge 0.5 --no-prepare --i-understand-writes
```

Put **global** flags (`--host`, `--timeout`, …) **before** the subcommand (`charge` / `discharge` / `read`); `argparse` does not accept `--timeout` after `charge` in this layout.

Use **`--max-kw`** to cap magnitude (default **5** kW). Use **`--no-prepare`** if your site is already in the correct instruction/control mode.

## Safety

- **Telemetry** uses **read-only** input registers (**FC 0x04**).
- **Charge / discharge** uses **FC 0x06** (`write_register`) on **holding** registers **102**, **104** (when prepare is enabled), and **105**. That can change real power flows. Misconfiguration, grid rules, or vendor limits can still block or fault the system—verify on your site and use small test powers first.
- Writes require **`--i-understand-writes`** on the CLI. There is no separate “stop power” command in this minimal helper; returning to vendor control or writing **0** to **105** (not implemented here by default) may be required—extend locally if needed.

## Protocol notes (from vendor files)

- **EMS-Telemetry**: summary power, voltage, current, SOC, temperatures, cell voltage extremes, **system status** (e.g. address **114**), **heartbeat** (**UINT32** at **136**).
- **EMS-control**: the normal telemetry output also reads holding register **104** as `ems_control_source` (**0 = local**, **1 = remote**, other = vendor-specific) to help diagnose whether Modbus control is selected.
- **BMS-Telemetry**: stack/cluster SOC, SOH, status, electrical and energy totals; **cluster SOC** at **2200** per PDF.
- **Scaling**: displayed values use workbook **Accuracy** (divisor) and **Offset** where applicable (e.g. power ÷ 10 for kW).
- **UINT32 on FOX**: two input registers may carry the value in the first word only, the second only (e.g. heartbeat), or both; the script uses a small heuristic (see `_decode_uint32_pair` in `fox_battery_modbus.py`). If totals look wrong on your firmware, compare with the vendor UI and adjust if needed.

## Files

| File | Purpose |
|------|---------|
| `fox_battery_modbus.py` | CLI: telemetry read (FC 0x04); optional `charge` / `discharge` (FC 0x06); helpers `charge_with_power_kw`, `discharge_with_power_kw` |
| `.env.example` | Template for `FOX_MODBUS_HOST` / `FOX_MODBUS_PORT` (copy to `.env`) |
| `requirements.txt` | `pymodbus` dependency |
| `*.xlsx`, `*.pdf` | Vendor protocol reference (not parsed at runtime) |
