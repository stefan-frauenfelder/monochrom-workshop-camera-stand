
import ST7789
from PIL import Image, ImageDraw, ImageFont

# external display pins
DISPLAY_SPI_PORT = 0
DISPLAY_SPI_CS = 0
DISPLAY_SPI_DC = 25
DISPLAY_BACKLIGHT = 7

# Screen dimensions
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 240


class Display:

    def __init__(self):

        self.display = ST7789.ST7789(
            port=DISPLAY_SPI_PORT,
            cs=DISPLAY_SPI_CS,
            dc=DISPLAY_SPI_DC,
            backlight=DISPLAY_BACKLIGHT,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            rotation=180,
            spi_speed_hz=60 * 1000 * 1000
        )

    def demo(self):
        # Clear the display to a red background.
        # Can pass any tuple of red, green, blue values (from 0 to 255 each).
        # Get a PIL Draw object to start drawing on the display buffer.
        img = Image.new('RGB', (DISPLAY_WIDTH, DISPLAY_HEIGHT), color=(255, 0, 0))

        draw = ImageDraw.Draw(img)

        # Draw a purple rectangle with yellow outline.
        draw.rectangle((10, 10, DISPLAY_WIDTH - 10, DISPLAY_HEIGHT - 10), outline=(255, 255, 0), fill=(255, 0, 255))

        # Draw some shapes.
        # Draw a blue ellipse with a green outline.
        draw.ellipse((10, 10, DISPLAY_WIDTH - 10, DISPLAY_HEIGHT - 10), outline=(0, 255, 0), fill=(0, 0, 255))

        # Draw a white X.
        draw.line((10, 10, DISPLAY_WIDTH - 10, DISPLAY_HEIGHT - 10), fill=(255, 255, 255))
        draw.line((10, DISPLAY_HEIGHT - 10, DISPLAY_WIDTH - 10, 10), fill=(255, 255, 255))

        # Draw a cyan triangle with a black outline.
        draw.polygon([(DISPLAY_WIDTH / 2, 10), (DISPLAY_WIDTH - 10, DISPLAY_HEIGHT - 10), (10, DISPLAY_HEIGHT - 10)], outline=(0, 0, 0), fill=(0, 255, 255))

        # Load default font.
        font = ImageFont.load_default()

        # Write two lines of white text on the buffer, rotated 90 degrees counter clockwise.
        self.draw_rotated_text(img, 'Hello World!', (0, 0), 90, font, fill=(255, 255, 255))
        self.draw_rotated_text(img, 'This is a line of text.', (10, DISPLAY_HEIGHT - 10), 0, font, fill=(255, 255, 255))

        # Write buffer to display hardware, must be called to make things visible on the
        # display!
        self.display.display(img)

    # Define a function to create rotated text.  Unfortunately PIL doesn't have good
    # native support for rotated fonts, but this function can be used to make a
    # text image and rotate it so it's easy to paste in the buffer.

    def draw_rotated_text(self, image, text, position, angle, font, fill=(255, 255, 255)):
        # Get rendered font width and height.
        draw = ImageDraw.Draw(image)
        width, height = draw.textsize(text, font=font)
        # Create a new image with transparent background to store the text.
        textimage = Image.new('RGBA', (width, height), (0, 0, 0, 0))
        # Render the text.
        textdraw = ImageDraw.Draw(textimage)
        textdraw.text((0, 0), text, font=font, fill=fill)
        # Rotate the text image.
        rotated = textimage.rotate(angle, expand=1)
        # Paste the text into the image, using it as a mask for transparency.
        image.paste(rotated, position, rotated)