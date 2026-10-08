import pytest

from canstepper.protocol import (
    Frame,
    PARAMS,
    Param,
    decode_line,
    decode_param_value,
    encode_line,
    encode_param_value,
    make_can_id,
    resolve_param,
    split_can_id,
)


def test_can_id_roundtrip():
    for node in (0, 1, 17, 31):
        for msg in (0, 6, 33, 63):
            can_id = make_can_id(node, msg)
            assert split_can_id(can_id) == (node, msg)


def test_can_id_validation():
    with pytest.raises(ValueError):
        make_can_id(32, 0)
    with pytest.raises(ValueError):
        make_can_id(1, 64)


def test_line_roundtrip_data_frame():
    frame = Frame.command(5, 4, bytes.fromhex("0000000000804640"))
    line = encode_line(frame)
    parsed = decode_line(line)
    assert parsed == frame
    assert parsed.node_id == 5 and parsed.msg_id == 4 and not parsed.rtr


def test_line_roundtrip_rtr():
    frame = Frame.rtr_request(9, 33)
    parsed = decode_line(encode_line(frame))
    assert parsed.rtr and parsed.node_id == 9 and parsed.msg_id == 33
    assert parsed.data == b""


def test_debug_and_garbage_lines_ignored():
    assert decode_line("# GrafitoCANStepper fw 1.0") is None
    assert decode_line("") is None
    assert decode_line("not hex at all") is None
    # USB-JTAG mashed lines seen on the live bench (must not become ghost nodes)
    assert decode_line("100 5100000") is None
    assert decode_line("0 00001811F260050000000") is None
    assert decode_line("A  0000AB70050000010") is None


def test_param_registry_complete_and_resolvable():
    for param in Param:
        d = PARAMS[param]
        assert d.param == param
        assert resolve_param(param.name.lower()) is d
        assert resolve_param(int(param)) is d


def test_param_value_roundtrip():
    d_float = PARAMS[Param.MAX_SPEED]
    raw = encode_param_value(d_float, 123.5)
    assert decode_param_value(d_float, raw) == pytest.approx(123.5)

    d_int = PARAMS[Param.MICROSTEPS]
    raw = encode_param_value(d_int, 64)
    assert decode_param_value(d_int, raw) == 64


def test_resolve_param_unknown_name():
    with pytest.raises(KeyError):
        resolve_param("warp_drive")
