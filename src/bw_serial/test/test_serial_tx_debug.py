from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / "src" / "mantis_comm_node.cpp").read_text(encoding="utf-8")


def test_serial_tx_debug_is_disabled_by_default_and_has_no_hex_format_loop():
    assert 'declare_parameter("serial_tx_debug_enabled", false)' in SOURCE
    assert "if (this->serial_tx_debug_enabled_)" in SOURCE
    assert "for (size_t i = 0; i < bytes_transmit_size; ++i)" not in SOURCE
