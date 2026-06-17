"""
aracer tcp raw data format:
實際上就是經過包裝的標準CANBUS包，每個包的格式如下：
每包19byte
ex:f801c00e00000182000801043a2a01f419012f
扣掉f801c00e0000 標頭
CAN包的範圍
"
CAN ID:00 00 01 82
困惑的佔位:00
DLC: 08
data:01 04 3a 2a 01 f4 19 01
"
checksum:2f

data的第一個byte是index
data00 index
data01~07 數據實際內容
按照實際數據的不同，占用位數也不一樣

checksum是LRC校驗碼

f8 01 c0 "0e"
0e是payload length
扣掉標頭後跟checksum的長度是0e=14byte
前面4byte是CANFDID
DLC應該是4個byte的CAN ID + 1 byte的佔位 + 1 byte的DLC + 8 byte的data = 14byte

"""

"""
to do:
vss、afr修正
減少無謂的格式轉換
"""
import time
import timeit
import ast
import datetime
import inspect
import math
import textwrap

ID = '0182'  # CAN ID (這是monitor的ID)
length = 19  # LL=0x0E monitor frame total length
FRAME_HEADER = bytes.fromhex("f801c0")
FRAME_LENGTH_OFFSET = 3
FRAME_OVERHEAD = 5
MAX_FRAME_LENGTH = 512

# 暫時性的，趕著比賽可用
# rear Tire setting
width = 120      # 輪胎寬度(mm)
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


# ===== 工具函式 =====
def nmea_to_decimal(nmea_str: str, is_lat: bool) -> float:
    """
    將 NMEA 格式的 DDMM.mmmm / DDDMM.mmmm 轉為十進位度數
    is_lat=True 表示緯度(通常 2 位數度)，False 表示經度(3 位數度)
    """
    if not nmea_str:
        return 0.0

    val = float(nmea_str)
    # 緯度/經度都可以用 //100 切出度數部分
    deg = int(val // 100)
    minutes = val - deg * 100
    return deg + minutes / 60.0


def checksum(cs):  # 計算NMEA0183校驗和
    checksum_val = 0
    for s in cs:
        checksum_val ^= ord(s)
    return '{:02X}'.format(checksum_val)


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


# ===== 航向計算 + EMA =====
class BearingCalculator:
    def __init__(self, speed_threshold_knots: float = 1.0, ema_alpha: float = 0.3):
        # 前一筆位置 (decimal degrees)
        self.prev_lat = None
        self.prev_lon = None

        # 上一次輸出的(平滑後)航向字串
        self.last_output = "0.00"

        # EMA 相關
        self.speed_threshold_knots = speed_threshold_knots
        self.ema_alpha = ema_alpha
        self.ema_value = None  # float, degrees

    def update(self, nmea_lat: str, lat_ns: str,
               nmea_lon: str, lon_ew: str,
               speed_knots: float | None) -> str:
        """
        使用上一點與這一點的位置計算 COG，並做 EMA 平滑
        nmea_lat, nmea_lon: NMEA 'DDMM.mmmm' / 'DDDMM.mmmm'
        lat_ns: 'N' 或 'S'
        lon_ew: 'E' 或 'W'
        speed_knots: 當下速度(節)，低於門檻不更新航向
        """
        # 速度太低時不更新航向，避免靜止時 GPS 抖動亂跳
        if speed_knots is not None and speed_knots < self.speed_threshold_knots:
            return self.last_output

        if not nmea_lat or not nmea_lon:
            return self.last_output

        # NMEA -> decimal degrees
        lat_deg = nmea_to_decimal(nmea_lat, is_lat=True)
        lon_deg = nmea_to_decimal(nmea_lon, is_lat=False)

        # 南半球 / 西經轉成負值
        if lat_ns == 'S':
            lat_deg = -lat_deg
        if lon_ew == 'W':
            lon_deg = -lon_deg

        # 第一次沒有前一筆資料：只記錄，不更新方向
        if self.prev_lat is None or self.prev_lon is None:
            self.prev_lat = lat_deg
            self.prev_lon = lon_deg
            return self.last_output

        # 換成弧度
        lat1 = math.radians(self.prev_lat)
        lon1 = math.radians(self.prev_lon)
        lat2 = math.radians(lat_deg)
        lon2 = math.radians(lon_deg)

        dlon = lon2 - lon1

        # 方位角公式
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        bearing_deg = (math.degrees(bearing) + 360.0) % 360.0

        # 記下這次位置，給下一筆用
        self.prev_lat = lat_deg
        self.prev_lon = lon_deg

        # EMA 平滑
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
    """
    獲取函數中指定變量的賦值表達式
    """
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
                # 這裡用 ast.unparse 把右邊的表達式還原成程式碼字串
                try:
                    expr_code = ast.unparse(node.value)
                except AttributeError:
                    # 舊版 Python 沒有 ast.unparse，就退而求其次簡單抓一行
                    lines = textwrap.dedent(source).splitlines()
                    line = node.value.lineno - 1
                    expr_code = lines[line].split('=', 1)[-1].strip()

                return f"{name} = {expr_code}"

    raise ValueError(f"Variable '{var_name}' not found in function '{func.__name__}'")


class EcuDecoder:
    def __init__(self):
        self.count = 0
        self.bearing_calc = BearingCalculator(speed_threshold_knots=1.0, ema_alpha=0.3)
        self.speed_acc = acceleration()
        self.rpm_acc = acceleration()
        self.rr_acc = acceleration()
        self.reset_values()

    def reset_values(self):
        self.gps_utc_hh = ""
        self.gps_utc_mm = ""
        self.gps_utc_ss = ""
        self.gps_utc_ms = ""
        self.gps_lat_deg = ""
        self.gps_lat_min = ""
        self.gps_lat_sec = ""
        self.gps_lat_ns = "N"
        self.gps_lon_deg = ""
        self.gps_lon_min = ""
        self.gps_lon_sec = ""
        self.gps_lon_ew = "E"
        self.gps_valid = "V"
        self.gps_speed = "0"
        self.gps_got_data = False
        self.rpm = 0
        self.tps = 0
        self.vss1 = 0
        self.vss2 = 0
        self.tc_lean_angle = 0
        self.tc_vss_fr_rate = 0
        self.volt_batt = 0
        self.t_eng = 0
        self.t_air = 0
        self.afr_wbo2_1 = 0
        self.afr_wbo2_2 = 0
        self.cyl1_eng_ap = 0
        self.cyl1_eng_ap_decimal = 0
        self.racelanuh_en = 0

    def decode_frame(self, message: bytes) -> str:
        if len(message) < 11 or message[6:8] != bytes.fromhex(ID):
            return ""

        idx = message[10]

        match idx:
            case 1:
                if len(message) < length:
                    return ""
                self.gps_utc_hh = f"{message[11]:02d}"
                self.gps_utc_mm = f"{message[12]:02d}"
                self.gps_utc_ss = f"{message[13]:02d}"
                self.gps_utc_ms = f"{int.from_bytes(message[14:16], byteorder='big'):03d}"
                self.gps_lat_deg = f"{message[16]:02d}"
                self.gps_lat_min = f"{message[17]:02d}"
                return ""

            case 2:
                if len(message) < length:
                    return ""
                self.gps_lat_sec = f"{int.from_bytes(message[11:13], byteorder='big'):04d}"
                self.gps_lat_ns = chr(message[13])
                self.gps_lon_deg = f"{message[14]:02d}"
                self.gps_lon_min = f"{message[15]:02d}"
                self.gps_lon_sec = f"{int.from_bytes(message[16:18], byteorder='big'):04d}"
                return ""

            case 3:
                if len(message) < 15:
                    return ""
                self.gps_lon_ew = chr(message[11])
                self.gps_valid = chr(message[12])
                self.gps_speed = f"{int.from_bytes(message[13:15], byteorder='big') * 0.539956803:.3f}"
                self.gps_got_data = True
                return ""

            case 4:
                if len(message) < length:
                    return ""
                self.rpm = int.from_bytes(message[14:16], byteorder='big')
                self.tps = message[16] / 255 * 100
                self.vss1 = message[17]
                return ""

            case 5:
                if len(message) < length:
                    return ""
                self.vss2 = message[11]
                self.tc_lean_angle = message[12] - 127
                self.tc_vss_fr_rate = message[13] * 0.78
                self.volt_batt = message[14] / 10
                self.t_eng = message[15] - 28
                self.t_air = message[16] - 28
                self.afr_wbo2_1 = message[17]
                return ""

            case 6:
                if len(message) < 15:
                    return ""
                self.afr_wbo2_2 = message[11]
                self.cyl1_eng_ap = message[12]
                self.cyl1_eng_ap_decimal = message[13]
                self.racelanuh_en = message[14]
                return ""

            case _:
                return self.build_output()

    def build_output(self) -> str:
        self.count += 1
        if self.count > 65535:
            self.count = 0

        has_gps = (
            self.gps_got_data and
            self.gps_utc_hh != "" and
            self.gps_lat_deg != "" and
            self.gps_lon_deg != "" and
            self.gps_valid == "A"
        )

        if has_gps:
            quality = 1
            nmea_time = f"{self.gps_utc_hh}{self.gps_utc_mm}{self.gps_utc_ss}.{self.gps_utc_ms}"
            date = datetime.datetime.now(datetime.UTC).strftime('%d%m%y')
            gps_lat = f"{self.gps_lat_deg}{self.gps_lat_min}.{self.gps_lat_sec}"
            gps_lon = f"{self.gps_lon_deg}{self.gps_lon_min}.{self.gps_lon_sec}"

            try:
                gps_speed_f = float(self.gps_speed)
            except Exception:
                gps_speed_f = 0.0

            bearing = self.bearing_calc.update(
                gps_lat, self.gps_lat_ns,
                gps_lon, self.gps_lon_ew,
                gps_speed_f
            )
        else:
            quality = 0
            nmea_time = ""
            date = ""
            gps_lat = ""
            gps_lon = ""
            gps_speed_f = 0.0
            bearing = "0.00"

        ms2 = self.speed_acc.calculate(gps_speed_f * 0.514444444)
        rpm_i = int(self.rpm) if self.rpm else 0
        rps2 = self.rpm_acc.calculate(rpm_i / 60 if rpm_i else 0.0)

        try:
            denom = (gps_speed_f * 30.8666667) / tire_circumference
            Reduction_Ratio = "0.000" if denom == 0 else f"{rpm_i / denom:.3f}"
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
                f"GNGGA,{nmea_time},{gps_lat},{self.gps_lat_ns},"
                f"{gps_lon},{self.gps_lon_ew},{quality},,,,M,,M,,"
            )
            RMC = (
                f"GNRMC,{nmea_time},{self.gps_valid},"
                f"{self.gps_lat_deg}{self.gps_lat_min}.{self.gps_lat_sec},{self.gps_lat_ns},"
                f"{self.gps_lon_deg}{self.gps_lon_min}.{self.gps_lon_sec},{self.gps_lon_ew},"
                f"{self.gps_speed},{bearing},{date},,,A"
            )
            result += f"${GGA}*{checksum(GGA)}\n"
            result += f"${RMC}*{checksum(RMC)}\n"

        RC3 = (
            f"RC3,{nmea_time},{self.count},,,,,,,"
            f"{self.rpm},{self.tps},{self.vss1},{self.vss2},{self.tc_lean_angle},{self.tc_vss_fr_rate},"
            f"{self.volt_batt},{self.t_eng},{self.t_air},{self.afr_wbo2_1},{self.afr_wbo2_2},"
            f"{self.cyl1_eng_ap},{self.cyl1_eng_ap_decimal},{self.racelanuh_en},"
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
    data = "f801c00e0000018200080103252402581706e3f801c00e0000018200080209854e780d0e1026f801c00e0000018200080345410020753371e5f801c00e00000182000804fa6d7f2bbaff27b2f801c00e00000182000805ff64008d704309f6f801c00e00000182000806fb5b2c09290b1cc6f801c00e00000182000807000000100000038df801c00e000001820006887e538300ff0000ce"
    byte_data = bytes.fromhex(data)
    value = convert(byte_data)
    print(value)
    print(get_variable_expr(convert, 'RC3'))

    print(timeit.timeit('convert(byte_data)', globals=globals(), number=10000))
