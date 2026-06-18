"""
aracer tcp raw data format:
實際上就是經過包裝的標準 CANBUS 包，每個包的格式如下：
每包 19 byte
ex:f801c00e00000182000801043a2a01f419012f

CAN ID:00 00 01 82
困惑的佔位:00
DLC: 08
data:01 04 3a 2a 01 f4 19 01

checksum 是 LRC 校驗碼
"""

import ast
import datetime
import inspect
import math
import os
import textwrap
import time
import timeit

ID = '0182'  # CAN ID (monitor)
length = 19  # LL=0x0E monitor frame total length
FRAME_HEADER = bytes.fromhex("f801c0")
FRAME_LENGTH_OFFSET = 3
FRAME_OVERHEAD = 5
MAX_FRAME_LENGTH = 512

# 暫時性的，趕著比賽可用
# rear Tire setting
width = 120  # 輪胎寬度(mm)
aspect_ratio = 80  # 輪胎側比(%)
rim_diameter = 12  # 輪圈直徑(inch)
# 計算輪胎圓周長(cm)
tire_circumference = ((width * aspect_ratio * 2 / 1000) + (rim_diameter * 2.54)) * math.pi / 100
# 齒比
font_gear_in = 12
font_gear_out = 39
rear_gear_in = 13
rear_gear_out = 40
gear_ratio = (font_gear_out / font_gear_in) * (rear_gear_out / rear_gear_in)

class V:
    GPS_UTC_hh = 111
    GPS_UTC_mm = 112
    GPS_UTC_ss = 113
    GPS_UTC_ms = 114
    GPS_Valid = 115
    GPS_Lat_deg = 116
    GPS_Lat_min = 117
    GPS_Lat_mmmm = 118
    GPS_Lat_NS = 119
    GPS_Lon_deg = 120
    GPS_Lon_min = 121
    GPS_Lon_mmmm = 122
    GPS_Lon_EW = 123
    GPS_Speed = 124
    RPM = 26
    TPS_Percent = 34
    Vehicle_Speed = 51
    TC_Lean_Angle = 108
    TC_FR_Rate = 126
    Volt_Batt = 29
    T_Eng = 33
    T_Air = 24
    AFR_WBO2 = 55
    Cyl1_Eng_AP = 45

SCALE = {
    V.RPM: (1, 0),
    V.TPS_Percent: (2.55, 0),
    V.Vehicle_Speed: (256, 0),
    V.TC_Lean_Angle: (1, 128),
    V.TC_FR_Rate: (1.28, 0),
    V.Volt_Batt: (10, 0),
    V.T_Eng: (1, 28),
    V.T_Air: (1, 28),
    V.AFR_WBO2: (100, 0),
    V.Cyl1_Eng_AP: (256, 0),
}


def aracer_checksum_ok(frame: bytes) -> bool:
    return (sum(frame[2:]) & 0xff) == 0


def extract_aracer_frames(buffer: bytes):
    frames = []

    while True:
        start = buffer.find(FRAME_HEADER)

        if start < 0:
            keep = len(FRAME_HEADER) - 1
            buffer = buffer[-keep:]
            break

        if start > 0:
            buffer = buffer[start:]

        if len(buffer) <= FRAME_LENGTH_OFFSET:
            break

        frame_length = buffer[FRAME_LENGTH_OFFSET] + FRAME_OVERHEAD
        if frame_length > MAX_FRAME_LENGTH:
            buffer = buffer[1:]
            continue

        if len(buffer) < frame_length:
            break

        frame = buffer[:frame_length]
        if not aracer_checksum_ok(frame):
            buffer = buffer[1:]
            continue

        frames.append(frame)
        buffer = buffer[frame_length:]

    return frames, buffer


def _parse_pdo_map(cfg_path):
    fields = {}
    off = 0
    try:
        with open(cfg_path, 'r') as f:
            lines = f.readlines()
    except OSError:
        return fields, 0

    for ln in lines:
        ln = ln.strip()
        if len(ln) != 38:
            continue
        try:
            b = bytes.fromhex(ln)
        except ValueError:
            continue
        if b[6:8] != b'\x06\x02':
            continue
        d = b[10:18]
        if not (d[0] == 0x23 and d[1] == 0x00 and d[2] == 0x1A):
            continue
        varid = (d[5] << 8) | d[6]
        nbytes = d[4] // 8
        fields[varid] = (off, nbytes)
        off += nbytes
    return fields, off


_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.txt')
FIELDS, CYCLE_BYTES = _parse_pdo_map(_cfg)
if not FIELDS:
    FIELDS = {
        111: (0, 1), 112: (1, 1), 113: (2, 1), 114: (3, 2), 116: (5, 1), 117: (6, 1),
        118: (7, 2), 119: (9, 1), 120: (10, 1), 121: (11, 1), 122: (12, 2), 123: (14, 1),
        115: (15, 1), 124: (16, 2), 26: (24, 2), 34: (26, 1), 51: (27, 2), 108: (29, 1),
        126: (30, 1), 29: (31, 1), 33: (32, 1), 24: (33, 1), 55: (34, 2), 45: (36, 2),
    }
    CYCLE_BYTES = 54

_last_off = max((o + n) for vid, (o, n) in FIELDS.items() if vid in SCALE or 110 < vid < 125)
EMIT_INDEX = (_last_off - 1) // 7 + 1


# ===== 工具函式 =====
def nmea_to_decimal(nmea_str: str, is_lat: bool) -> float:
    if not nmea_str:
        return 0.0
    val = float(nmea_str)
    deg = int(val // 100)
    minutes = val - deg * 100
    return deg + minutes / 60.0


def checksum(cs):
    checksum_val = 0
    for s in cs:
        checksum_val ^= ord(s)
    return '{:02X}'.format(checksum_val)


# ===== 航向計算 + EMA =====
class BearingCalculator:
    def __init__(self, speed_threshold_knots: float = 1.0, ema_alpha: float = 0.3):
        self.prev_lat = None
        self.prev_lon = None
        self.last_output = "0.00"
        self.speed_threshold_knots = speed_threshold_knots
        self.ema_alpha = ema_alpha
        self.ema_value = None

    def update(self, nmea_lat, lat_ns, nmea_lon, lon_ew, speed_knots):
        if speed_knots is not None and speed_knots < self.speed_threshold_knots:
            return self.last_output
        if not nmea_lat or not nmea_lon:
            return self.last_output
        lat_deg = nmea_to_decimal(nmea_lat, is_lat=True)
        lon_deg = nmea_to_decimal(nmea_lon, is_lat=False)
        if lat_ns == 'S':
            lat_deg = -lat_deg
        if lon_ew == 'W':
            lon_deg = -lon_deg
        if self.prev_lat is None or self.prev_lon is None:
            self.prev_lat = lat_deg
            self.prev_lon = lon_deg
            return self.last_output
        lat1 = math.radians(self.prev_lat)
        lon1 = math.radians(self.prev_lon)
        lat2 = math.radians(lat_deg)
        lon2 = math.radians(lon_deg)
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        bearing_deg = (math.degrees(bearing) + 360.0) % 360.0
        self.prev_lat = lat_deg
        self.prev_lon = lon_deg
        if self.ema_value is None:
            self.ema_value = bearing_deg
        else:
            alpha = self.ema_alpha
            self.ema_value = alpha * bearing_deg + (1.0 - alpha) * self.ema_value
        self.last_output = f"{self.ema_value:.2f}"
        return self.last_output


# ===== 加速度計算 =====
class acceleration:
    def __init__(self):
        self.time = time.time()
        self.speed = 0.0

    def calculate(self, speed: float) -> str:
        current_time = time.time()
        dt = current_time - self.time
        speed_diff = speed - self.speed
        self.speed = speed
        self.time = current_time
        if dt == 0:
            return "0.000"
        acceleration_val = speed_diff / dt
        return f"{acceleration_val:.3f}"


def get_variable_expr(func, var_name):
    """獲取函數中指定變量的賦值表達式（main.py 用來印出 RC3 格式）"""
    if func.__name__ == 'convert' and var_name == 'RC3':
        func = EcuDecoder.build_output

    source = textwrap.dedent(inspect.getsource(func))
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name):
                name = target.id
            elif isinstance(target, ast.Attribute):
                name = target.attr
            else:
                continue
            if name == var_name:
                try:
                    expr_code = ast.unparse(node.value)
                except AttributeError:
                    lines = textwrap.dedent(source).splitlines()
                    line = node.value.lineno - 1
                    expr_code = lines[line].split('=', 1)[-1].strip()
                return f"{name} = {expr_code}"
    raise ValueError(f"Variable '{var_name}' not found in function '{func.__name__}'")


class EcuDecoder:
    def __init__(self):
        self.count = 0
        self.buf = bytearray(max(CYCLE_BYTES, 64))
        self.bearing_calc = BearingCalculator(speed_threshold_knots=1.0, ema_alpha=0.3)
        self.speed_acc = acceleration()
        self.rpm_acc = acceleration()
        self.rr_acc = acceleration()

    def raw(self, varid):
        fo = FIELDS.get(varid)
        if fo is None:
            return None
        o, nb = fo
        if o + nb > len(self.buf):
            return None
        return int.from_bytes(self.buf[o:o + nb], 'big')

    def phy(self, varid, default=0.0):
        raw = self.raw(varid)
        if raw is None:
            return default
        gain, offset = SCALE.get(varid, (1, 0))
        return (raw - offset) / gain

    def decode_frame(self, message: bytes) -> str:
        if len(message) < 11 or message[6:8] != bytes.fromhex(ID):
            return ""

        idx = message[10]
        if idx == 0 or idx > 16:
            return ""

        dlc = message[9]
        payload = message[11:11 + max(dlc - 1, 0)]
        base = (idx - 1) * 7
        for k, byte in enumerate(payload):
            if base + k < len(self.buf):
                self.buf[base + k] = byte

        if idx != EMIT_INDEX:
            return ""

        return self.build_output()

    def build_output(self) -> str:
        self.count = (self.count + 1) & 0xFFFF

        def s2(v):
            r = self.raw(v)
            return r if r is not None else 0

        gps_utc_hh = f"{s2(V.GPS_UTC_hh):02d}"
        gps_utc_mm = f"{s2(V.GPS_UTC_mm):02d}"
        gps_utc_ss = f"{s2(V.GPS_UTC_ss):02d}"
        gps_utc_ms = f"{s2(V.GPS_UTC_ms):03d}"
        gps_lat_deg = f"{s2(V.GPS_Lat_deg):02d}"
        gps_lat_min = f"{s2(V.GPS_Lat_min):02d}"
        gps_lat_sec = f"{s2(V.GPS_Lat_mmmm):04d}"
        gps_lat_ns = chr(s2(V.GPS_Lat_NS)) if 32 <= s2(V.GPS_Lat_NS) < 127 else 'N'
        gps_lon_deg = f"{s2(V.GPS_Lon_deg):02d}"
        gps_lon_min = f"{s2(V.GPS_Lon_min):02d}"
        gps_lon_sec = f"{s2(V.GPS_Lon_mmmm):04d}"
        gps_lon_ew = chr(s2(V.GPS_Lon_EW)) if 32 <= s2(V.GPS_Lon_EW) < 127 else 'E'
        gps_valid = chr(s2(V.GPS_Valid)) if 32 <= s2(V.GPS_Valid) < 127 else 'V'
        gps_speed_kmh = float(s2(V.GPS_Speed))
        gps_speed = f"{gps_speed_kmh * 0.539956803:.3f}"

        rpm = int(self.phy(V.RPM))
        tps = self.phy(V.TPS_Percent)
        veh_speed = self.phy(V.Vehicle_Speed)
        lean_angle = self.phy(V.TC_Lean_Angle)
        fr_rate = self.phy(V.TC_FR_Rate)
        volt_batt = self.phy(V.Volt_Batt)
        t_eng = self.phy(V.T_Eng)
        t_air = self.phy(V.T_Air)
        afr_wbo2 = self.phy(V.AFR_WBO2)
        map_kpa = self.phy(V.Cyl1_Eng_AP)

        has_gps = (gps_valid == 'A')

        if has_gps:
            nmea_time = f"{gps_utc_hh}{gps_utc_mm}{gps_utc_ss}.{gps_utc_ms}"
            date = datetime.datetime.now(datetime.UTC).strftime('%d%m%y')
            gps_lat = f"{gps_lat_deg}{gps_lat_min}.{gps_lat_sec}"
            gps_lon = f"{gps_lon_deg}{gps_lon_min}.{gps_lon_sec}"
            bearing = self.bearing_calc.update(
                gps_lat, gps_lat_ns, gps_lon, gps_lon_ew, gps_speed_kmh * 0.539956803)
        else:
            nmea_time = ""
            date = ""
            bearing = "0.00"

        ms2 = self.speed_acc.calculate(gps_speed_kmh * 0.277777778)
        rps2 = self.rpm_acc.calculate(rpm / 60 if rpm else 0.0)
        try:
            denom = (gps_speed_kmh * 1000 / 60) / tire_circumference if tire_circumference else 0
            Reduction_Ratio = f"{rpm / denom:.3f}" if denom else "0.000"
        except Exception:
            Reduction_Ratio = "0.000"
        try:
            irrs2 = self.rr_acc.calculate(float(Reduction_Ratio))
        except Exception:
            irrs2 = "0.000"
        try:
            alpha_ratio = f"{float(Reduction_Ratio) / gear_ratio:.3f}"
        except Exception:
            alpha_ratio = "0.000"

        result = ""
        if has_gps:
            GGA = (
                f"GNGGA,{nmea_time},{gps_lat},{gps_lat_ns},"
                f"{gps_lon},{gps_lon_ew},1,,,,M,,M,,"
            )
            RMC = (
                f"GNRMC,{nmea_time},{gps_valid},"
                f"{gps_lat_deg}{gps_lat_min}.{gps_lat_sec},{gps_lat_ns},"
                f"{gps_lon_deg}{gps_lon_min}.{gps_lon_sec},{gps_lon_ew},"
                f"{gps_speed},{bearing},{date},,,A"
            )
            result += f"${GGA}*{checksum(GGA)}\n"
            result += f"${RMC}*{checksum(RMC)}\n"

        RC3 = (
            f"RC3,{nmea_time},{self.count},,,,,,,"
            f"{rpm},{tps:.1f},{veh_speed:.2f},{lean_angle:.0f},{afr_wbo2:.2f},"
            f"{map_kpa:.1f},{volt_batt:.1f},{t_eng:.0f},{t_air:.0f},{fr_rate:.1f},"
            f"{Reduction_Ratio},{alpha_ratio},{irrs2},{ms2},{rps2}"
        )
        result += f"${RC3}*{checksum(RC3)}\n"
        return result


def convert(data: bytes) -> str:
    if not hasattr(convert, 'framer_buffer'):
        convert.framer_buffer = b""
    if not hasattr(convert, 'decoder'):
        convert.decoder = EcuDecoder()

    convert.framer_buffer += data
    frames, convert.framer_buffer = extract_aracer_frames(convert.framer_buffer)
    results = []

    for message in frames:
        result = convert.decoder.decode_frame(message)
        if result:
            results.append(result)

    return results[-1] if results else ""


if __name__ == '__main__':
    print("PDO Mapping 佈局（VarID: offset, bytes）:")
    for vid, (o, n) in sorted(FIELDS.items(), key=lambda kv: kv[1][0]):
        print(f"  VarID {vid:>4} @ byte {o:>2} ({n}B)")
    print(f"一輪 {CYCLE_BYTES} bytes，EMIT_INDEX = {EMIT_INDEX}")
    print()

    data = (
        "f801c00e0000018200080103252402581706e3"
        "f801c00e0000018200080209854e780d0e1026"
        "f801c00e0000018200080345410020753371e5"
        "f801c00e00000182000804fa6d7f2bbaff27b2"
        "f801c00e00000182000805ff64008d704309f6"
        "f801c00e00000182000806fb5b2c09290b1cc6"
        "f801c00e00000182000807000000100000038d"
        "f801c00e000001820006887e538300ff0000ce"
    )
    byte_data = bytes.fromhex(data)
    value = convert(byte_data)
    print(value)
    print(get_variable_expr(convert, 'RC3'))
    print(timeit.timeit('convert(byte_data)', globals=globals(), number=10000))
