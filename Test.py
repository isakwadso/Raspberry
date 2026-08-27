cat > blink.py << 'EOF'
from gpiozero import LED, Button
from time import sleep
import gpiozero as gpio
import time


led = LED(18)
button = Button(25)

while True:
    if button.is_pressed:
        led.off()
    else:
        led.on()
    sleep(0.05)
EOF

#
