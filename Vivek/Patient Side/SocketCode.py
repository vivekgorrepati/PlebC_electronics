import websocket
import threading

class WebSocketClient:
    def __init__(self, websocket_url):
        self.websocket_url = websocket_url
        self.ws = websocket.WebSocketApp(self.websocket_url)
        #self.ws.on_message = self.on_message
        self.ws.on_open = self.on_open
        self.ws.on_close = self.on_close
        self.send_lock = threading.Lock()

    def on_message(self, _, message):
        print("Received message:", message)

    def on_open(self, _):
        print("Connection opened")
        self.send_message("Hello, server!")

    def on_close(self, _):
        print("Connection closed")

    def send_message(self, message):
        with self.send_lock:
            self.ws.send(message)

    def start(self):
        ws_thread = threading.Thread(target=self.ws.run_forever)
        ws_thread.start()

        """
        while True:
            user_input = input("Enter a message (or 'exit' to quit): ")
            
            if user_input == "exit":
                break
            
            self.send_message(user_input)

        self.ws.close()
        ws_thread.join()
        """


