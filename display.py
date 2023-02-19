
import ST7789
from PIL import Image, ImageOps, ImageDraw, ImageFont

# external display pins
DISPLAY_SPI_PORT = 0
DISPLAY_SPI_CS = 0
DISPLAY_SPI_DC = 25
DISPLAY_BACKLIGHT = 7

# native screen dimensions
HARDWARE_WIDTH = 320
HARDWARE_HEIGHT = 240

# all drawing happens in an image that is 3 times larger and rotated
# resulting in a resolution of 720 width and 960 height
SCALING_FACTOR = 3
WIDTH = HARDWARE_HEIGHT * SCALING_FACTOR
HEIGHT = HARDWARE_WIDTH * SCALING_FACTOR

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)


class Display:

    NN_1050_50 = ImageFont.truetype(font='NN1050.otf', size=50)
    NN_1050_100 = ImageFont.truetype(font='NN1050.otf', size=100)

    def __init__(self):

        self.display = ST7789.ST7789(
            port=DISPLAY_SPI_PORT,
            cs=DISPLAY_SPI_CS,
            dc=DISPLAY_SPI_DC,
            backlight=DISPLAY_BACKLIGHT,
            width=HARDWARE_WIDTH,
            height=HARDWARE_HEIGHT,
            rotation=0,
            spi_speed_hz=30 * 1000 * 1000  # was 60, 40 works, 50 does not
        )

        self.image = Image.new('RGBA', (WIDTH, HEIGHT), color=BLACK)
        self.draw = ImageDraw.Draw(self.image)

    def show(self):
        # resize to fit hardware using antialiasing (which gives a nicer image than using the display's native size)
        rotated_hardware_image = self.image.resize((HARDWARE_HEIGHT, HARDWARE_WIDTH), Image.ANTIALIAS)
        # rotate the image since the ST7789 library wants it that way
        hardware_image = rotated_hardware_image.rotate(90, expand=True)
        # update the display using the resulted image
        self.display.display(hardware_image)

    def mode(self, text):
        # preferences
        y_location = 800
        label_font = self.NN_1050_50
        mode_font = self.NN_1050_100
        # draw a small gray label and a large white mode text centered on screen
        (mode_width, mode_height) = self.draw.textsize(text, font=mode_font)
        (label_width, label_height) = self.draw.textsize('Current mode:', font=label_font)
        left_space = (WIDTH - mode_width) / 2
        self.draw.text((left_space, y_location), 'Current mode:', font=label_font, fill=GRAY)
        self.draw.text((left_space, y_location + label_height + 20), text, font=mode_font, fill=WHITE)

    def clear(self):
        # draw black rectangle to clear
        self.draw.rectangle((0, 0, WIDTH, HEIGHT), fill=BLACK)

    def update(self):
        self.show()