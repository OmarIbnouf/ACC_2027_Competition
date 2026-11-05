from quanser.devices import Aaaf5050McK12LED


class PhysicalLEDs:
    def __init__(self):
        try:
            self.leds = Aaaf5050McK12LED()
            self.num_leds = 33
            self.leds.open(
                "spi://localhost:1?memsize=420,word=8,baud=3333333,lsb=off,frame=1",
                self.num_leds,
            )
            self.connected = True
        except Exception:
            self.connected = False

    def set_color(self, r, g, b):
        if self.connected:
            self.leds.writeColors((r, g, b), self.num_leds)

    def close(self):
        if self.connected:
            self.leds.writeColors((0, 0, 0), self.num_leds)
            self.leds.close()
