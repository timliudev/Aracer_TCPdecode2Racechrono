from twisted.internet import reactor, protocol
from twisted.internet.protocol import DatagramProtocol
import sys
from datetime import datetime

import decoode

# 以下是一些參數設置
config_file = 'config.txt'
timeout = 10  # 設置超時時間
default_ecu_ip = "192.168.88.88"  # 預設ECU IP位置
ecu_port = 6666  # ECU端口
ecu_udp_port = 8888  # ECU UDP broadcast端口
ecu_find_str = b'aRacer'  # ECU發現的關鍵字+
log_switch = False  # 日誌開關
FRAME_HEADER = bytes.fromhex("f801c0")
FRAME_LENGTH_OFFSET = 3
FRAME_OVERHEAD = 5  # header, length byte, and checksum
MAX_FRAME_LENGTH = 512


# print_with_time的時候加上時間
def print_with_time(*args, **kwargs):
    time = datetime.now().strftime('%H:%M:%S.%f')
    print(f"{time} ", *args, **kwargs)
    if log_switch:  # 如果日誌開關打開
        console_log.write(f"{time} " + ' '.join(map(str, args)) + '\n')


# 讀取 config.txt 檔案
def read_config():
    with open(config_file, 'r') as f:
        lines = f.readlines()

    # 將 watchdog 的值設為變數並轉換成 bytes
    watchdog = bytes.fromhex(lines[1].strip())

    # 將 init 內容設為陣列並轉換成 bytes
    init = [bytes.fromhex(line.strip()) for line in lines[4:]]

    return watchdog, init


def aracer_checksum_ok(frame):
    return (sum(frame[2:]) & 0xff) == 0


class HeaderFramer:
    def __init__(self, header, frame_length=None):
        self.header = header
        self.frame_length = frame_length
        self.buffer = b""

    def feed(self, data):
        self.buffer += data
        frames = []

        while True:
            start = self.buffer.find(self.header)

            if start < 0:
                keep = len(self.header) - 1
                self.buffer = self.buffer[-keep:]
                break

            if start > 0:
                self.buffer = self.buffer[start:]

            if self.frame_length is None:
                if len(self.buffer) <= FRAME_LENGTH_OFFSET:
                    break

                frame_length = self.buffer[FRAME_LENGTH_OFFSET] + FRAME_OVERHEAD
                if frame_length > MAX_FRAME_LENGTH:
                    self.buffer = self.buffer[1:]
                    continue
            else:
                frame_length = self.frame_length

            if len(self.buffer) < frame_length:
                break

            frame = self.buffer[:frame_length]
            if not aracer_checksum_ok(frame):
                self.buffer = self.buffer[1:]
                continue

            frames.append(frame)
            self.buffer = self.buffer[frame_length:]

        return frames


# 這個class是用來處理ECU UDP發現的
class EcuUDPdiscover(DatagramProtocol):
    def __init__(self):
        self.timeout_deferred = None  # 初始化timeout_deferred屬性

    # 連線成功後執行
    def startProtocol(self):
        self.timeout_deferred = reactor.callLater(timeout, self.timeout)  # 設定超時時間

    # 接收數據
    def datagramReceived(self, datagram, address):
        if ecu_find_str in datagram:  # 如果收到了關鍵字
            print_with_time(f"find ECU at {address}")
            ecuConnect(address[0])  # 連接ECU
            self.transport.stopListening()  # 停止監聽
            if self.timeout_deferred.active():  # 如果收到了期望的關鍵字，取消超時操作
                self.timeout_deferred.cancel()

    # 超時操作
    def timeout(self):
        print_with_time("DiscoverTimeout，connect to default ECU IP")
        self.transport.stopListening()  # 停止監聽
        ecuConnect(default_ecu_ip)  # 連接預設ECU IP位置


# 建立新的UDP接收器
def ecuUDPdiscoverStart():
    udp = EcuUDPdiscover()
    reactor.listenUDP(ecu_udp_port, udp)
    print_with_time("Discover ECU...")


# 連接ECU
def ecuConnect(ecu_ip):
    print_with_time('connect ecu ip:', ecu_ip)
    reactor.connectTCP(ecu_ip, ecu_port, EcuClientFactory())  # 連接ECU


# 這個class是用來處理ECU通訊的
class EcuClient(protocol.Protocol):
    # 初始化config.txt的內容
    def __init__(self):
        self.watchdog, self.init = read_config()  # 讀取config.txt的內容
        self.framer = HeaderFramer(FRAME_HEADER)
        self.decoder = decoode.EcuDecoder()
        self.watchdog_call = None
        self.init_call = None
        self.connected = False

    # 連線成功後執行
    def connectionMade(self):
        print_with_time("Connected to the ecu.")
        self.connected = True
        self.transport.setTcpNoDelay(True)  # 關閉 Nagle 算法
        self.send_init()  # 發送初始化數據
        self.watchdog_call = reactor.callLater(1, self.send_watchdog)  # 每1秒發送一次 watchdog

    def connectionLost(self, reason):
        self.connected = False
        for call in (self.watchdog_call, self.init_call):
            if call is not None and call.active():
                call.cancel()

    # 發送初始化數據
    def send_init(self):
        if self.init:
            item = self.init.pop(0)
            self.transport.write(item)
            self.init_call = reactor.callLater(0.01, self.send_init)

    # 發送 watchdog
    def send_watchdog(self):
        if not self.connected:
            return
        self.transport.write(self.watchdog)
        self.watchdog_call = reactor.callLater(1, self.send_watchdog)

    # 接收數據
    def dataReceived(self, data):
        for frame in self.framer.feed(data):
            message = self.decoder.decode_frame(frame)
            if message:
                broadcast(message)

        # 將數據寫入日誌
        self.log_ecu(data)

    # 日誌回調函數
    def log_ecu(self, data):
        if log_switch:  # 如果日誌開關打開
            ecu_log.write(f"{datetime.now()},{data.hex()}\n")  # 寫入日誌


# 這個class是用來創建EcuClient的
class EcuClientFactory(protocol.ReconnectingClientFactory):
    protocol = EcuClient  # 設置protocol屬性為EcuClient

    # 創建protocol
    def buildProtocol(self, addr):
        self.resetDelay()  # 重置延遲
        return self.protocol()

    def clientConnectionFailed(self, connector, reason):
        print_with_time(f"Connection failed to {connector.getDestination()}")  # 連接失敗
        if self.continueTrying:
            self.connector = connector
            self.retry()

    def clientConnectionLost(self, connector, unused_reason):
        print_with_time(f"Connection lost to {connector.getDestination()}")  # 連接丟失
        if self.continueTrying:
            self.connector = connector
            self.retry()


# 這個class是用來處理RC3server通訊的
class RC3server(protocol.Protocol):
    # 連線成功後執行
    def connectionMade(self):
        print_with_time("Client connected")
        if hasattr(self, 'factory'):  # 如果有factory屬性
            self.factory.clients.append(self)  # 將自己添加到clients列表中

    # 連線失敗後執行
    def connectionLost(self, reason):
        print_with_time("Client Connection lost")
        if hasattr(self, 'factory'):  # 如果有factory屬性
            self.factory.clients.remove(self)  # 將自己從clients列表中移除


# 這個class是用來創建RC3server的
class RC3serverFactory(protocol.Factory):
    # 初始化
    def __init__(self):
        self.clients = []  # 創建一個空的clients列表

    # 創建protocol
    def buildProtocol(self, addr):
        protocol_instance = RC3server()  # 創建一個RC3server實例
        protocol_instance.factory = self  # 设置factory属性
        return protocol_instance  # 返回protocol實例


# 向所有客戶端廣播數據
def broadcast(message):
    if not message:
        return
    clients = factory.clients  # 獲取clients列表
    for client in clients:  # 遍歷clients列表
        client.transport.write(message.encode())  # 向每個客戶端發送數據
    log_rc3(message)


# 日誌回調函數
def log_rc3(data):
    if log_switch:  # 如果日誌開關打開
        rc3_log.write(f"{datetime.now()},{data}\n")  # 寫入日誌


if __name__ == '__main__':

    if "-l" in sys.argv:  # 如果命令行參數中有-l，開啟日誌紀錄
        log_switch = True
        date = datetime.now().strftime("%Y%m%d%H%M%S")
        console_log = open(f"{date}_console_log.txt", 'w')
        ecu_log = open(f"{date}_ECU_log.txt", 'w')
        rc3_log = open(f"{date}_RC3_log.txt", 'w')
        print_with_time("Log switch on")

    ecuUDPdiscoverStart()  # 啟動ECU UDP發現

    factory = RC3serverFactory()  # 創建RC3serverFactory實例
    reactor.listenTCP(7777, factory)  # 監聽7777端口

    print_with_time("Aracer SuperX ECU Wifi protocol to RaceChrono RC3 server started.")
    print_with_time(decoode.get_variable_expr(decoode.convert, 'RC3'))

    reactor.run()
