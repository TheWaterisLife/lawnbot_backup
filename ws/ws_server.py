import asyncio
import json
import websockets

PORT = 8765
MOTOR_SERVER_URL = "ws://127.0.0.1:8766"


async def forward_to_motor(cmd: str):
    """
    Forward motor command ("on"/"off") to motor test server on port 8766
    """
    try:
        async with websockets.connect(MOTOR_SERVER_URL) as motor_ws:
            await motor_ws.send(cmd)
            print(f"Forwarded motor command: {cmd}")
    except Exception as e:
        print("Motor forward error:", e)


async def handler(ws):
    print("Client connected")

    try:
        # Initial handshake message
        await ws.send("hello from pi")

        async for msg in ws:
            print("RX:", msg)

            # Try to parse JSON
            try:
                obj = json.loads(msg)
            except Exception:
                obj = None

            # Handle motor commands
            if isinstance(obj, dict) and obj.get("type") == "motor":
                cmd = str(obj.get("cmd", "")).lower()

                if cmd in ("on", "off"):
                    await forward_to_motor(cmd)
                    await ws.send(json.dumps({
                        "type": "ack",
                        "motor": cmd
                    }))
                    continue

            # Default behavior: echo everything else
            await ws.send(f"echo: {msg}")

    except websockets.exceptions.ConnectionClosed:
        pass

    finally:
        print("Client disconnected")


async def main():
    async with websockets.serve(handler, "0.0.0.0", PORT):
        print(f"WebSocket server listening on 0.0.0.0:{PORT}")
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())

