"""PLC-facing CANopen e2e: every object a master will touch, in PLC order.

These tests are the contract for Codesys / TwinCAT / TIA import of
GrafitoCANStepper.eds. They run against the in-process slave that matches
firmware 2.1.
"""

from __future__ import annotations

import struct

import pytest

from canstepper.canopen402 import (
    CW_NEW_SETPOINT,
    CW_RELATIVE,
    DType,
    ERR_BLOCKED,
    EREG_GENERIC,
    EREG_PROFILE,
    NmtState,
    OD_ENTRIES,
    OpMode,
    SDO_ABORT_NO_SUB,
    SDO_ABORT_VALUE,
    SDO_ABORT_WO,
    SW_OPERATION_ENABLED,
    SW_QUICK_STOP_ACTIVE,
    SW_READY_TO_SWITCH_ON,
    SW_SWITCH_ON_DISABLED,
    SW_SWITCHED_ON,
    SW_TARGET_REACHED,
    CanFrame,
    CanOpenMaster,
    CanOpenSlave,
    DriveState,
    HomingMethod,
    NmtCommand,
    OdEntry,
    VirtualCanBus,
    cob_heartbeat,
    cob_rpdo2,
    cob_rsdo,
    cob_tpdo2,
    cob_tsdo,
    counts_from_deg,
    encode_sdo_expedited_download,
    encode_sdo_upload_request,
    od_entry,
    parse_eds,
    parse_sdo,
    eds_path,
)


def _net(node_id: int = 1):
    bus = VirtualCanBus()
    slave = CanOpenSlave(node_id, bus)
    master = CanOpenMaster(bus, node_id)
    return bus, slave, master


def _writable(entry: OdEntry) -> bool:
    if entry.access != "rw":
        return False
    # Drive / identity-adjacent values that have side effects or live COB-IDs
    if entry.index in (0x6040, 0x2000) or entry.index in range(0x1400, 0x1C00):
        return False
    return True


def test_plc_reads_every_eds_object_via_sdo():
    _bus, _slave, master = _net()
    failures = []
    for entry in OD_ENTRIES.values():
        if entry.access == "wo":
            continue
        try:
            value = master.sdo_read(entry.index, entry.sub)
        except Exception as exc:
            failures.append(f"0x{entry.index:04X}:{entry.sub:02X} {entry.name}: {exc}")
            continue
        if entry.dtype == DType.VIS:
            assert isinstance(value, str) and value
        elif entry.dtype == DType.R32:
            assert value == pytest.approx(float(entry.default), rel=1e-5, abs=1e-5)
        else:
            assert int(value) == int(entry.default) or entry.index in (
                0x6041,  # live statusword
                0x1014, 0x1200, 0x1400, 0x1401, 0x1800, 0x1801,  # node-scaled COB-IDs
            )
    assert not failures, "SDO catalogue failed:\n" + "\n".join(failures)


def test_plc_write_readback_commissioning_objects():
    _bus, _slave, master = _net()
    samples = {
        (0x1017, 0): 500,
        (0x605A, 0): 2,
        (0x6060, 0): 3,
        (0x607A, 0): counts_from_deg(12.5),
        (0x6081, 0): 2275,
        (0x6083, 0): 13107,
        (0x6084, 0): 13107,
        (0x6085, 0): 26214,
        (0x6098, 0): int(HomingMethod.ENDSTOP_POS),
        (0x6099, 1): 1000,
        (0x6099, 2): 200,
        (0x609A, 0): 20000,
        (0x60FF, 0): -500,
        (0x2001, 0): 200,
        (0x2002, 0): 8,
        (0x2003, 0): 35,
        (0x2004, 0): 8,
        (0x2005, 0): 20,
        (0x2006, 0): 1,
        (0x2007, 0): 0,
        (0x200A, 0): 0,
        (0x200B, 0): 2,
        (0x200C, 0): 0,
        (0x200D, 0): 0,
        (0x200E, 0): 500_000,
        (0x200F, 0): 9.5,
        (0x2010, 0): 0.25,
        (0x2011, 0): 0.05,
        (0x2012, 0): 0.5,
        (0x2013, 0): 0,
        (0x2014, 0): 1,
        (0x2015, 0): 15000,
    }
    for (index, sub), value in samples.items():
        master.sdo_write(index, sub, value)
        got = master.sdo_read(index, sub)
        if od_entry(index, sub).dtype == DType.R32:
            assert got == pytest.approx(float(value), rel=1e-5)
        else:
            assert int(got) == int(value)


def test_illegal_commissioning_values_abort():
    bus, _slave, _master = _net()

    def abort_code(index, raw):
        bus.send(CanFrame(cob_rsdo(1), encode_sdo_expedited_download(index, 0, raw)))
        reply = bus.take(cob_tsdo(1))[-1]
        cmd, _i, _s, payload = parse_sdo(reply.data)
        assert cmd == 0x80
        return struct.unpack("<I", payload)[0]

    assert abort_code(0x2002, struct.pack("<I", 12)) == SDO_ABORT_VALUE  # not power of two
    assert abort_code(0x200E, struct.pack("<I", 800_000)) == SDO_ABORT_VALUE
    assert abort_code(0x6060, struct.pack("<b", 7)) == SDO_ABORT_VALUE  # interpolated not supported
    bus.send(CanFrame(cob_rsdo(1), encode_sdo_upload_request(0x1018, 9)))
    reply = bus.take(cob_tsdo(1))[-1]
    assert parse_sdo(reply.data)[0] == 0x80
    assert struct.unpack("<I", reply.data[4:8])[0] == SDO_ABORT_NO_SUB


def test_write_only_save_and_defaults():
    bus, slave, master = _net()
    master.sdo_write(0x2003, 0, 44)
    master.sdo_write(0x2008, 0, 1)
    assert slave.saved is True
    bus.send(CanFrame(cob_rsdo(1), encode_sdo_upload_request(0x2008, 0)))
    data = bus.take(cob_tsdo(1))[-1].data
    assert parse_sdo(data)[0] == 0x80
    assert struct.unpack("<I", data[4:8])[0] == SDO_ABORT_WO
    master.sdo_write(0x2009, 0, 1)
    assert master.sdo_read(0x2003) == 20


def test_full_402_state_walk_for_plc():
    _bus, slave, master = _net()
    assert master.sdo_read(0x6041) & 0x006F == SW_SWITCH_ON_DISABLED
    master.sdo_write(0x6040, 0, 0x0006)
    assert master.sdo_read(0x6041) & 0x006F == SW_READY_TO_SWITCH_ON
    master.sdo_write(0x6040, 0, 0x0007)
    assert master.sdo_read(0x6041) & 0x006F == SW_SWITCHED_ON
    master.sdo_write(0x6040, 0, 0x000F)
    assert master.sdo_read(0x6041) & 0x006F == SW_OPERATION_ENABLED
    master.sdo_write(0x6040, 0, 0x0007)  # disable operation
    assert slave.drive.state == DriveState.SWITCHED_ON
    master.sdo_write(0x6040, 0, 0x000F)
    master.sdo_write(0x6040, 0, 0x0006)  # shutdown from OE
    assert slave.drive.state == DriveState.READY_TO_SWITCH_ON
    master.sdo_write(0x6040, 0, 0x0007)
    master.sdo_write(0x6040, 0, 0x000F)
    master.sdo_write(0x6040, 0, 0x0000)  # disable voltage
    assert slave.drive.state == DriveState.SWITCH_ON_DISABLED


def test_relative_profile_position():
    _bus, _slave, master = _net()
    master.enable_operation()
    master.move_absolute(counts_from_deg(20))
    master.move_relative(counts_from_deg(15))
    assert master.sdo_read(0x6064) == counts_from_deg(35)


def test_rpdo2_velocity_and_tpdo2():
    bus, slave, master = _net()
    master.enable_operation()
    master.sdo_write(0x6060, 0, int(OpMode.PROFILE_VELOCITY))
    master.start()
    bus.drain()
    master.send_rpdo2(0x000F, 4000)
    assert slave.values[(0x60FF, 0)] == 4000
    assert slave.values[(0x606C, 0)] == 4000
    slave.tick(20)
    tpdo2 = master.last_tpdo2()
    assert tpdo2 is not None
    sw, vel = tpdo2
    assert vel == 4000
    assert sw & 0x006F == SW_OPERATION_ENABLED


def test_nmt_stop_blocks_sdo_and_pdo():
    bus, slave, master = _net()
    master.nmt(NmtCommand.STOP)
    assert slave.nmt == NmtState.STOPPED
    bus.send(CanFrame(cob_rsdo(1), encode_sdo_upload_request(0x1000, 0)))
    assert bus.take(cob_tsdo(1)) == []
    master.send_rpdo1(0x000F | CW_NEW_SETPOINT, 1234)
    assert slave.values[(0x6064, 0)] == 0
    master.start()
    assert master.sdo_read(0x1000)


def test_nmt_addressing_ignores_other_node():
    bus, slave, master = _net(5)
    master.nmt(NmtCommand.START, dest=1)
    assert slave.nmt == NmtState.PRE_OPERATIONAL
    master.nmt(NmtCommand.START, dest=5)
    assert slave.nmt == NmtState.OPERATIONAL
    bus.send(CanFrame(cob_rsdo(4), encode_sdo_upload_request(0x1000, 0)))
    assert bus.take(cob_tsdo(5)) == []


def test_heartbeat_follows_nmt_and_period():
    bus, slave, master = _net()
    master.sdo_write(0x1017, 0, 100)
    bus.drain()
    slave.tick(100)
    hb = bus.take(cob_heartbeat(1))
    assert hb[-1].data == bytes((NmtState.PRE_OPERATIONAL,))
    master.start()
    bus.drain()
    slave.tick(200)
    hb = bus.take(cob_heartbeat(1))
    assert hb[-1].data == bytes((NmtState.OPERATIONAL,))


def test_eds_pdo_mapping_matches_slave():
    sections = parse_eds(eds_path().read_text(encoding="ascii"))
    assert sections["1600sub1"]["DefaultValue"].lower() == "0x60400010"
    assert sections["1600sub2"]["DefaultValue"].lower() == "0x607a0020"
    assert sections["1601sub2"]["DefaultValue"].lower() == "0x60ff0020"
    assert sections["1A00sub1"]["DefaultValue"].lower() == "0x60410010"
    assert sections["1A00sub2"]["DefaultValue"].lower() == "0x60640020"
    assert sections["1A01sub2"]["DefaultValue"].lower() == "0x606c0020"
    _bus, _slave, master = _net()
    assert master.sdo_read(0x1600, 1) == 0x60400010
    assert master.sdo_read(0x1600, 2) == 0x607A0020
    assert master.sdo_read(0x1A00, 2) == 0x60640020


def test_plc_golden_path_scan_enable_pp_pv_hm_fault():
    """One test that mirrors a PLC first-day program."""
    bus, slave, master = _net()
    # scan
    assert master.sdo_read(0x1000) == 0x00040192
    assert master.sdo_read(0x6502) == 0x25
    # commission
    master.sdo_write(0x200C, 0, 0)
    master.sdo_write(0x2003, 0, 30)
    master.sdo_write(0x6081, 0, 2275)
    master.sdo_write(0x6060, 0, 1)
    # nmt + 402
    master.start()
    master.enable_operation()
    assert slave.nmt == NmtState.OPERATIONAL
    assert master.sdo_read(0x6041) & 0x006F == SW_OPERATION_ENABLED
    # pp
    master.move_absolute(counts_from_deg(90))
    assert master.sdo_read(0x6064) == counts_from_deg(90)
    assert master.sdo_read(0x6041) & SW_TARGET_REACHED
    # pv
    master.set_velocity(1000)
    assert master.sdo_read(0x606C) == 1000
    master.sdo_write(0x6040, 0, 0x010F)
    assert master.sdo_read(0x606C) == 0
    # hm
    master.home(int(HomingMethod.CURRENT_POSITION))
    assert master.sdo_read(0x6064) == 0
    # pdo — mode must be pp again; 402 does not infer mode from RPDO contents
    master.sdo_write(0x6060, 0, int(OpMode.PROFILE_POSITION))
    bus.drain()
    master.send_rpdo1(0x000F | CW_NEW_SETPOINT | CW_RELATIVE, counts_from_deg(5))
    assert slave.values[(0x6064, 0)] == counts_from_deg(5)
    # fault + reset + re-enable
    slave.raise_fault(ERR_BLOCKED, EREG_PROFILE | EREG_GENERIC)
    master.sdo_write(0x6040, 0, 0x0080)
    assert slave.drive.state == DriveState.SWITCH_ON_DISABLED
    master.enable_operation()
    assert slave.drive.state == DriveState.OPERATION_ENABLED
    master.sdo_write(0x6040, 0, 0x0002)
    assert master.sdo_read(0x6041) & 0x006F == SW_QUICK_STOP_ACTIVE
    master.sdo_write(0x6040, 0, 0x0000)
    assert slave.drive.state == DriveState.SWITCH_ON_DISABLED
