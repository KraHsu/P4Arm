import socket
import threading
import json
import os


class TCPConnection:
    def __init__(self, host="127.0.0.1", port=8080):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_server = False
        self.lock = threading.Lock()
        self.handlers = {}  # Mapping of cmd to handler functions

    def start_server(self):
        self.is_server = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)

        def accept_connections():
            while True:
                client_socket, client_address = self.server_socket.accept()
                threading.Thread(
                    target=self.handle_client, args=(client_socket,)
                ).start()

        threading.Thread(target=accept_connections, daemon=True).start()

    def handle_client(self, client_socket):
        with client_socket:
            buffer = ""
            while True:
                try:
                    # 接收数据
                    data = client_socket.recv(1024)
                    if not data:
                        break

                    # 将接收到的数据解码并追加到缓冲区
                    buffer += data.decode("utf-8")

                    # 按行（以 '\n' 分隔）处理完整的消息
                    while "\n" in buffer:
                        # 提取一条完整消息
                        message_str, buffer = buffer.split("\n", 1)

                        try:
                            # 解析 JSON 数据
                            message = json.loads(message_str)

                            # 提取字段
                            cmd = message.get("cmd")
                            code = message.get("code")
                            payload = message.get("payload")

                            # 根据命令处理
                            if cmd in self.handlers:
                                response_payload = self.handlers[cmd](code, payload)
                                response = {
                                    "cmd": f"{cmd}_res",
                                    "code": 0,  # 0 for success
                                    "payload": response_payload,
                                }
                            else:
                                response = {
                                    "cmd": f"{cmd}_res",
                                    "code": 1,  # 1 for unknown command
                                    "payload": {"error": "Unknown command"},
                                }
                        except json.JSONDecodeError:
                            # 如果消息无法解析为 JSON
                            response = {
                                "cmd": "error",
                                "code": 2,  # 2 for JSON decode error
                                "payload": {"error": "Invalid JSON format"},
                            }
                            raise "Invalid JSON received"

                        # 发送响应，确保以 '\n' 结尾
                        client_socket.send(
                            (json.dumps(response) + "\n").encode("utf-8")
                        )

                except Exception as e:
                    raise f"Error handling client: {e}"

    def start_client(self):
        self.is_server = False
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port))

    def send_message(self, cmd, code, payload):
        if not self.client_socket:
            raise ConnectionError("Client is not connected to a server.")

        message = {"cmd": cmd, "code": code, "payload": payload}

        try:
            with self.lock:
                self.client_socket.send((json.dumps(message) + "\n").encode("utf-8"))
                response = self.client_socket.recv(10240)
                return json.loads(response.decode("utf-8"))
        except Exception as e:
            raise (f"Error sending message: {e}")

    def connect(self, cmd, handler):
        self.handlers[cmd] = handler

    def close(self):
        if self.server_socket:
            self.server_socket.close()
        if self.client_socket:
            self.client_socket.close()
        raise ("Connection closed.")
