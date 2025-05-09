import asyncio
from bleak import BleakClient
import requests

# ESP32 BLE MAC 주소와 특성 UUID
ESP32_ADDRESS = "XX:XX:XX:XX:XX:XX"  # ESP32의 BLE MAC 주소
CHARACTERISTIC_UUID = "abcdef01-1234-5678-1234-56789abcdef0"  # 특성 UUID
SERVER_URL = "http://your-server.com/api/ble-status"  # 서버 URL

def send_to_server(status, weight=None):
    payload = {"status": status}
    if weight is not None:
        payload["weight"] = weight
    try:
        response = requests.post(SERVER_URL, json=payload)
        if response.status_code == 200:
            print("📤 서버로 데이터 전송 성공!")
        else:
            print(f"❌ 서버 전송 실패: {response.status_code}")
    except Exception as e:
        print(f"❌ 서버 전송 중 오류 발생: {e}")

async def main():
    async with BleakClient(ESP32_ADDRESS) as client:
        print("🔗 BLE 연결 성공:", await client.is_connected())
        send_to_server("connected")  # 서버에 연결 상태 전송

        try:
            while True:
                # BLE 특성 데이터 읽기
                data = await client.read_gatt_char(CHARACTERISTIC_UUID)
                weight = float.fromhex(data.hex())  # 바이너리 데이터를 float로 변환
                print(f"📥 수신된 무게 데이터: {weight:.2f} g")

                # 서버로 데이터 전송
                send_to_server("connected", weight)

                await asyncio.sleep(1)  # 1초 간격으로 데이터 읽기
        except Exception as e:
            print(f"❌ 데이터 읽기 오류: {e}")
            send_to_server("disconnected")

asyncio.run(main())