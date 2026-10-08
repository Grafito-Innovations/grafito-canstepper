"""End-to-end CANopen / CiA 402 tests against the reference slave + EDS."""

from __future__ import annotations

import struct

import pytest

from canstepper.canopen402 import (
    CW_NEW_SETPOINT,
    DEVICE_NAME,
    DEVICE_TYPE,
    ENC_CPR,
    ERR_ENCODER,
    EREG_GENERIC,
    EREG_PROFILE,
    HW_VERSION,
    NmtState,
    OD_ENTRIES,
    OpMode,
    PRODUCT_CODE,
    REVISION,
    SDO_ABORT_NO_OBJECT,
    SDO_ABORT_RO,
    SW_OPERATION_ENABLED,
    SW_REMOTE,
    SW_SP_ACK_HOMED,
    SW_SWITCH_ON_DISABLED,
    SW_TARGET_REACHED,
    SW_VERSION,
    VENDOR_ID,
    CanFrame,
    CanOpenMaster,
    CanOpenSlave,
    DriveState,
    HomingMethod,
    NmtCommand,
    VirtualCanBus,
    cob_emcy,
    cob_heartbeat,
    cob_rpdo1,
    cob_rsdo,
    cob_tpdo1,
    cob_tpdo2,
    counts_from_deg,
    deg_from_counts,
    dcf_path,
    eds_object_indexes,
    eds_path,
    encode_sdo_expedited_download,
    encode_sdo_upload_request,
    instance_values,
    od_catalog,
    od_csv_path,
    od_json_path,
    parse_eds,
    parse_sdo,
    render_dcf,
    render_eds,
    render_od_csv,
    render_od_json,
    write_canopen_descriptions,
    write_eds,
)


def _network(node_id: int = 1):
    bus = VirtualCanBus()
    slave = CanOpenSlave(node_id, bus)
    master = CanOpenMaster(bus, node_id)
    return bus, slave, master


def test_bootup_heartbeat_and_preop():
    bus, slave, master = _network(4)
    frames = bus.take(cob_heartbeat(4))
    assert frames and frames[0].data == bytes((NmtState.BOOTUP,))
    assert slave.nmt == NmtState.PRE_OPERATIONAL
    master.start()
    assert slave.nmt == NmtState.OPERATIONAL
    master.preop()
    assert slave.nmt == NmtState.PRE_OPERATIONAL
    slave.tick(1000)
    hb = bus.take(cob_heartbeat(4))
    assert hb and hb[-1].data == bytes((NmtState.PRE_OPERATIONAL,))


def test_identity_sdo_matches_eds_constants():
    _bus, _slave, master = _network()
    assert master.sdo_read(0x1000) == DEVICE_TYPE
    assert master.sdo_read(0x1018, 1) == VENDOR_ID
    assert master.sdo_read(0x1018, 2) == PRODUCT_CODE
    assert master.sdo_read(0x1018, 3) == REVISION
    assert master.sdo_read(0x1008) == DEVICE_NAME
    assert master.sdo_read(0x1009) == HW_VERSION
    assert master.sdo_read(0x100A) == SW_VERSION
    assert master.sdo_read(0x6502) == 0x00000025
    assert master.sdo_read(0x2016) == 0x0200


def test_sdo_unknown_object_aborts():
    bus, _slave, _master = _network()
    bus.send(CanFrame(cob_rsdo(1), encode_sdo_upload_request(0x9999, 0)))
    reply = bus.take(0x580 + 1)[-1]
    cmd, index, sub, payload = parse_sdo(reply.data)
    assert cmd == 0x80 and index == 0x9999 and sub == 0
    assert struct.unpack("<I", payload)[0] == SDO_ABORT_NO_OBJECT


def test_sdo_write_to_readonly_aborts():
    bus, _slave, _master = _network()
    bus.send(CanFrame(cob_rsdo(1), encode_sdo_expedited_download(0x1000, 0, b"\x01\x00\x00\x00")))
    reply = bus.take(0x580 + 1)[-1]
    cmd, index, _sub, payload = parse_sdo(reply.data)
    assert cmd == 0x80 and index == 0x1000
    assert struct.unpack("<I", payload)[0] == SDO_ABORT_RO


def test_sdo_param_roundtrip_and_save():
    _bus, slave, master = _network()
    master.sdo_write(0x2003, 0, 40)
    assert master.sdo_read(0x2003) == 40
    master.sdo_write(0x200F, 0, 15.5)
    assert master.sdo_read(0x200F) == pytest.approx(15.5, rel=1e-6)
    master.sdo_write(0x2008, 0, 1)
    assert slave.saved is True
    with pytest.raises(RuntimeError, match="0x06090030"):
        master.sdo_write(0x200E, 0, 123456)


def test_cia402_enable_sequence_and_statusword():
    _bus, slave, master = _network()
    sw = master.sdo_read(0x6041)
    assert sw & 0x006F == SW_SWITCH_ON_DISABLED
    assert sw & SW_REMOTE
    master.sdo_write(0x6040, 0, 0x0006)
    assert slave.drive.state == DriveState.READY_TO_SWITCH_ON
    master.sdo_write(0x6040, 0, 0x0007)
    assert slave.drive.state == DriveState.SWITCHED_ON
    master.sdo_write(0x6040, 0, 0x000F)
    assert slave.drive.state == DriveState.OPERATION_ENABLED
    assert master.sdo_read(0x6041) & 0x006F == SW_OPERATION_ENABLED


def test_profile_position_move_end_to_end():
    _bus, slave, master = _network()
    master.enable_operation()
    target = counts_from_deg(180.0)
    master.move_absolute(target)
    assert master.sdo_read(0x6064) == target
    assert deg_from_counts(master.sdo_read(0x6064)) == pytest.approx(180.0)
    sw = master.sdo_read(0x6041)
    assert sw & SW_TARGET_REACHED
    assert sw & SW_SP_ACK_HOMED
    assert slave.drive.state == DriveState.OPERATION_ENABLED


def test_profile_velocity_and_halt():
    _bus, slave, master = _network()
    master.enable_operation()
    master.set_velocity(32768)
    assert master.sdo_read(0x606C) == 32768
    assert master.sdo_read(0x6061) == int(OpMode.PROFILE_VELOCITY)
    master.sdo_write(0x6040, 0, 0x010F)  # halt
    assert master.sdo_read(0x606C) == 0
    assert master.sdo_read(0x6041) & SW_TARGET_REACHED


def test_homing_current_position():
    _bus, slave, master = _network()
    master.enable_operation()
    master.move_absolute(counts_from_deg(90))
    master.home(int(HomingMethod.CURRENT_POSITION))
    assert master.sdo_read(0x6064) == 0
    assert master.sdo_read(0x6041) & SW_SP_ACK_HOMED
    assert master.sdo_read(0x6061) == int(OpMode.HOMING)


def test_pdo_position_command_in_operational():
    bus, slave, master = _network()
    master.enable_operation()
    master.start()
    bus.drain()
    target = counts_from_deg(45)
    master.send_rpdo1(0x000F | CW_NEW_SETPOINT, target)
    assert slave.values[(0x6064, 0)] == target
    slave.tick(20)
    tpdo = master.last_tpdo1()
    assert tpdo is not None
    sw, pos = tpdo
    assert pos == target
    assert sw & SW_TARGET_REACHED


def test_pdo_ignored_in_preoperational():
    _bus, slave, master = _network()
    master.enable_operation()  # still pre-op NMT
    assert slave.nmt == NmtState.PRE_OPERATIONAL
    master.send_rpdo1(0x000F | CW_NEW_SETPOINT, 999)
    assert slave.values[(0x6064, 0)] == 0


def test_fault_emcy_and_reset():
    bus, slave, master = _network()
    master.enable_operation()
    slave.raise_fault(ERR_ENCODER, EREG_PROFILE | EREG_GENERIC)
    emcy = bus.take(cob_emcy(1))
    assert emcy
    code, ereg = struct.unpack_from("<HB", emcy[-1].data)
    assert code == ERR_ENCODER
    assert ereg & EREG_GENERIC
    assert slave.drive.state == DriveState.FAULT
    master.sdo_write(0x6040, 0, 0x0080)
    assert slave.drive.state == DriveState.SWITCH_ON_DISABLED
    assert master.sdo_read(0x603F) == 0


def test_nmt_broadcast_reset_sends_bootup():
    bus, slave, master = _network(2)
    bus.drain()
    master.nmt(NmtCommand.RESET_NODE, dest=0)
    boot = bus.take(cob_heartbeat(2))
    assert boot and boot[-1].data == bytes((NmtState.BOOTUP,))
    assert slave.nmt == NmtState.PRE_OPERATIONAL


def test_quick_stop():
    _bus, slave, master = _network()
    master.enable_operation()
    master.set_velocity(1000)
    master.sdo_write(0x6040, 0, 0x0002)
    assert slave.drive.state == DriveState.QUICK_STOP_ACTIVE
    assert master.sdo_read(0x606C) == 0


def test_eds_file_covers_object_dictionary():
    path = write_eds()
    assert path == eds_path()
    text = path.read_text(encoding="ascii")
    assert text == render_eds()
    sections = parse_eds(text)
    assert sections["DeviceInfo"]["VendorNumber"].lower() == f"0x{VENDOR_ID:08x}"
    assert sections["DeviceInfo"]["ProductName"] == "Grafito CANStepper C3"
    assert sections["DeviceInfo"]["BaudRate_1000"] == "1"
    assert int(sections["DeviceInfo"]["NrOfRXPDO"]) == 2
    assert int(sections["DeviceInfo"]["NrOfTXPDO"]) == 2
    indexes = set(eds_object_indexes(sections))
    od_indexes = {entry.index for entry in OD_ENTRIES.values()}
    assert indexes == od_indexes
    # PLC-facing 402 objects used in the enable + move sequence
    for section in ("6040", "6041", "6060", "6064", "607A", "60FF", "6098", "1018sub1"):
        assert section in sections
    assert sections["6040"]["PDOMapping"] == "1"
    assert sections["1000"]["AccessType"] == "ro"
    assert sections["2008"]["AccessType"] == "wo"


def test_dcf_and_object_dictionary_match_eds():
    paths = write_canopen_descriptions(node_id=1)
    assert paths["dcf"] == dcf_path()
    assert paths["od_json"] == od_json_path()
    assert paths["od_csv"] == od_csv_path()

    dcf = parse_eds(dcf_path().read_text(encoding="ascii"))
    eds = parse_eds(eds_path().read_text(encoding="ascii"))
    assert dcf["FileInfo"]["LastEDS"] == "GrafitoCANStepper.eds"
    assert dcf["DeviceComissioning"]["NodeID"] == "1"
    assert dcf["DeviceComissioning"]["Baudrate"] == "1000"
    assert set(eds_object_indexes(dcf)) == set(eds_object_indexes(eds))
    assert dcf["1000"]["ParameterValue"] == eds["1000"]["DefaultValue"]
    assert dcf["2000"]["ParameterValue"] == "1"
    assert dcf["1400sub1"]["ParameterValue"] == f"0x{cob_rpdo1(1):X}"
    assert "ParameterValue" not in eds["6040"]

    node4 = parse_eds(render_dcf(node_id=4))
    assert node4["DeviceComissioning"]["NodeID"] == "4"
    assert node4["2000"]["ParameterValue"] == "4"
    assert node4["1801sub1"]["ParameterValue"] == f"0x{cob_tpdo2(4):X}"
    assert node4["1400sub1"]["DefaultValue"] == eds["1400sub1"]["DefaultValue"]

    catalog = od_catalog(1)
    assert len(catalog) == len(OD_ENTRIES)
    assert {row["index_int"] for row in catalog} == {e.index for e in OD_ENTRIES.values()}
    values = instance_values(1)
    assert values[(0x2000, 0)] == 1
    assert values[(0x1800, 1)] == cob_tpdo1(1)

    payload = __import__("json").loads(od_json_path().read_text(encoding="utf-8"))
    assert payload["device"]["product_name"] == "Grafito CANStepper C3"
    assert payload["device"]["node_id"] == 1
    assert len(payload["objects"]) == len(OD_ENTRIES)
    csv_text = od_csv_path().read_text(encoding="utf-8")
    assert csv_text == render_od_csv()
    assert "0x6040" in csv_text
    assert render_od_json() == od_json_path().read_text(encoding="utf-8")


def test_encoder_count_units():
    assert counts_from_deg(360.0) == ENC_CPR
    assert counts_from_deg(90.0) == ENC_CPR // 4
    assert deg_from_counts(ENC_CPR) == pytest.approx(360.0)


def test_firmware_stack_matches_python_identity():
    header = (
        eds_path().parent / "canopen_stack.h"
    ).read_text(encoding="utf-8")
    sketch = (
        eds_path().parent / "GrafitoCANStepper_C3_CANopen.ino"
    ).read_text(encoding="utf-8")
    assert "0x000005A3" in header
    assert "0x00004333" in header
    assert "0x00040192" in header
    assert 'CO_DEVICE_NAME[] = "CANStepper"' in header
    assert "static const uint8_t FW_MAJOR = 2" in sketch
    assert "CiA402" in sketch
