import asyncio
from bleak import BleakClient, BleakScanner
import socket

# --- 設定區 ---
UDP_IP = "172.19.24.160"
UDP_PORT = 9001
DEVICE_NAME = "WT901BLE67"
# --------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def notification_handler(sender, data):
    sock.sendto(data, (UDP_IP, UDP_PORT))
    print(f"傳輸中... 原始數據: {data.hex()[:20]}", end='\r')

async def main():
    print(f"正在搜尋藍牙設備: {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    
    if not device:
        print("錯誤: 找不到設備，請確認 IMU 已開啟。")
        return

    print(f"找到設備 {device.address}，嘗試連線...")
    async with BleakClient(device) as client:
        print(f"連線成功！正在尋找數據頻道...")
        
        # 自動搜尋含有 'notify' 或 'read' 權限的 UUID
        target_uuid = None
        for service in client.services:
            for char in service.characteristics:
                # 維特通常使用具有 notify 屬性的特徵值
                if "notify" in char.properties:
                    target_uuid = char.uuid
                    print(f"找到可用頻道: {target_uuid} (屬性: {char.properties})")
                    break
            if target_uuid: break

        if not target_uuid:
            print("錯誤: 找不到可用的數據頻道 (Characteristic)。")
            return

        await client.start_notify(target_uuid, notification_handler)
        print(f"開始轉發數據至 WSL2 (Port: {UDP_PORT})...")
        
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"\n程式發生錯誤: {e}")
    except KeyboardInterrupt:
        print("\n使用者停止程式。")