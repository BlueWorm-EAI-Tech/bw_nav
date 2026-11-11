import hashlib
import ipaddress
import re
import subprocess
from pathlib import Path


def _is_private_ipv4(ip: str) -> bool:
    """判断是否为可用于局域网发现的私有 IPv4 地址。"""
    try:
        addr = ipaddress.ip_address(ip)
    except ValueError:
        return False

    return (
        isinstance(addr, ipaddress.IPv4Address)
        and addr.is_private
        and not addr.is_loopback
        and not addr.is_link_local
    )


def _get_private_ip_by_interface(iface: str):
    """从指定网卡读取私有 IPv4 地址。"""
    try:
        output = subprocess.check_output(
            ["ip", "-4", "addr", "show", "dev", iface],
            encoding="utf-8",
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None

    for ip in re.findall(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/\d+", output):
        if _is_private_ipv4(ip):
            return ip
    return None


def _get_ip_from_main_default_route():
    """
    优先读取 main 路由表的默认出口地址，避免被策略路由/虚拟网卡误导。
    """
    try:
        output = subprocess.check_output(
            ["ip", "-4", "route", "show", "default"],
            encoding="utf-8",
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None

    for line in output.splitlines():
        text = line.strip()
        if not text:
            continue

        src_match = re.search(r"\bsrc\s+(\d+\.\d+\.\d+\.\d+)\b", text)
        if src_match:
            src_ip = src_match.group(1)
            if _is_private_ipv4(src_ip):
                return src_ip

        dev_match = re.search(r"\bdev\s+(\S+)\b", text)
        if dev_match:
            iface_ip = _get_private_ip_by_interface(dev_match.group(1))
            if iface_ip:
                return iface_ip

    return None


def _get_wifi_interfaces() -> list[str]:
    """获取 WiFi 网卡列表（仅无线网卡）。"""
    wifi_ifaces = []
    try:
        for iface_path in Path("/sys/class/net").iterdir():
            iface = iface_path.name
            if (iface_path / "wireless").exists():
                wifi_ifaces.append(iface)
    except Exception:
        pass

    # 兜底：若未识别到 wireless 目录，则按常见 WiFi 前缀过滤
    if not wifi_ifaces:
        try:
            output = subprocess.check_output(
                ["ip", "-o", "link", "show"],
                encoding="utf-8",
                stderr=subprocess.DEVNULL,
            )
            for line in output.splitlines():
                match = re.search(r"^\d+:\s+([^:]+):", line.strip())
                if not match:
                    continue
                iface = match.group(1).split("@", 1)[0]
                if iface.startswith(("wl", "wlan")):
                    wifi_ifaces.append(iface)
        except Exception:
            pass

    return sorted(set(wifi_ifaces))


def _get_wifi_ip_from_ifconfig() -> str | None:
    """在缺少 ip 命令或 /sys 无线信息时，使用 ifconfig 兜底读取 WiFi IP。"""
    try:
        output = subprocess.check_output(
            ["ifconfig"],
            encoding="utf-8",
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None

    current_iface = None
    block_lines: list[str] = []

    def _pick_ip(iface: str | None, lines: list[str]) -> str | None:
        if not iface or not iface.startswith(("wl", "wlan")):
            return None
        text = "\n".join(lines)
        match = re.search(r"\binet\s(?:addr:)?(\d+\.\d+\.\d+\.\d+)\b", text)
        if not match:
            return None
        ip = match.group(1)
        return ip if _is_private_ipv4(ip) else None

    for line in output.splitlines():
        raw = line.rstrip("\n")
        if raw and not raw[0].isspace():
            picked = _pick_ip(current_iface, block_lines)
            if picked:
                return picked

            current_iface = raw.split(":", 1)[0].strip()
            block_lines = [raw]
        else:
            block_lines.append(raw)

    return _pick_ip(current_iface, block_lines)


def get_robot_sn():
    """
    仅使用 WiFi 网卡 MAC 计算 SN。
    不依赖 product_uuid，不做本地写入缓存。
    """
    wifi_ifaces = _get_wifi_interfaces()
    if not wifi_ifaces:
        raise RuntimeError("无法读取 WiFi 网卡。请确认存在无线网卡并可访问 /sys/class/net。")

    wifi_mac = None
    for iface in wifi_ifaces:
        try:
            with open(
                f"/sys/class/net/{iface}/address",
                "r",
                encoding="utf-8",
                errors="ignore",
            ) as f:
                mac = f.read().strip().lower()
        except Exception:
            continue

        # 过滤无效 MAC
        if not mac or mac == "00:00:00:00:00:00" or mac == "ff:ff:ff:ff:ff:ff":
            continue
        if not re.fullmatch(r"[0-9a-f]{2}(:[0-9a-f]{2}){5}", mac):
            continue

        wifi_mac = mac
        break

    if not wifi_mac:
        raise RuntimeError("无法读取有效的 WiFi MAC 地址。")

    digest = hashlib.md5(wifi_mac.encode("utf-8")).digest()

    # 8位 Base36（0-9A-Z）
    alphabet = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    number = int.from_bytes(digest[:8], byteorder="big", signed=False)
    chars = []
    for _ in range(8):
        number, rem = divmod(number, 36)
        chars.append(alphabet[rem])
    chars.reverse()
    suffix = "".join(chars)

    # 强制混合：至少包含 1 个数字 + 1 个字母
    has_digit = any(ch.isdigit() for ch in suffix)
    has_alpha = any(ch.isalpha() for ch in suffix)
    if not has_digit:
        suffix = suffix[:-1] + str(digest[8] % 10)
    if not has_alpha:
        suffix = chr(ord("A") + (digest[9] % 26)) + suffix[1:]

    return f"BW_{suffix}"


def get_local_ip():
    """
    仅从 WiFi 网卡获取本机局域网 IP。
    不使用有线网卡与外网探测。
    """
    for iface in _get_wifi_interfaces():
        ip = _get_private_ip_by_interface(iface)
        if ip:
            return ip

    # 兜底：某些容器镜像没有 ip 命令，但有 ifconfig
    ifconfig_ip = _get_wifi_ip_from_ifconfig()
    if ifconfig_ip:
        return ifconfig_ip

    return "127.0.0.1"

if __name__ == "__main__":
    print(get_robot_sn())
