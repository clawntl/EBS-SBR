#!/usr/bin/env python3
"""Generate scan-to-join Wi-Fi QR codes for a BalanceBot.

Each robot's access point is named from the last bytes of its soft-AP MAC
address (see firmware/src/WebInterface.cpp), e.g.

    SSID:     BalanceBot-5302cc
    Password: balancebot-5302cc

The suffix is printed on the serial console at boot:

    [wifi] AP "BalanceBot-5302cc" (password "balancebot-5302cc") up at 192.168.4.1

Usage:
    python tools/wifi_qr.py 5302cc
    python tools/wifi_qr.py BalanceBot-5302cc a1b2c3 --out signs/

For each robot this writes (to tools/qr-codes/ unless --out is given;
that folder is git-ignored as the files contain the robot's password):
    BalanceBot-<suffix>-wifi-qr.png  printable sign (QR + caption)
    BalanceBot-<suffix>-wifi-qr.svg  bare QR code, scales to any size

Requires qrcode + Pillow, e.g. in a virtual environment:
    python3 -m venv .venv && source .venv/bin/activate
    pip install "qrcode[pil]"
"""

import argparse
import re
import sys
from pathlib import Path

try:
    import qrcode
    import qrcode.image.svg
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit('Missing dependencies - run: python3 -m pip install "qrcode[pil]" '
             '(inside a venv if your Python is externally managed)')

# Keep in sync with AP_SSID_PREFIX / AP_PASSWORD_PREFIX in WebInterface.cpp.
SSID_PREFIX = "BalanceBot-"
PASSWORD_PREFIX = "balancebot-"

DEFAULT_OUT = Path(__file__).resolve().parent / "qr-codes"

SIGN_WIDTH = 1240  # px; ~10.5 cm at 300 dpi
MARGIN = 40

# Bold fonts to try, in order (Linux, macOS, Windows). Falls back to Pillow's
# built-in font if none are found.
FONT_CANDIDATES = [
    "DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
    "/Library/Fonts/Arial Bold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
]


def parse_suffix(arg: str) -> str:
    """Accept '5302cc', '5302CC' or 'BalanceBot-5302cc'; return '5302cc'."""
    s = arg.strip()
    if s.lower().startswith(SSID_PREFIX.lower()):
        s = s[len(SSID_PREFIX):]
    s = s.lower()
    if not re.fullmatch(r"(?:[0-9a-f]{2}){1,6}", s):
        raise argparse.ArgumentTypeError(
            f"'{arg}' is not a MAC suffix (expected hex like 5302cc)")
    return s


def wifi_escape(value: str) -> str:
    """Escape characters that are special in the WIFI: QR payload."""
    return re.sub(r'([\\;,:"])', r"\\\1", value)


def wifi_payload(ssid: str, password: str) -> str:
    return f"WIFI:T:WPA;S:{wifi_escape(ssid)};P:{wifi_escape(password)};;"


def load_font(size: int) -> ImageFont.ImageFont:
    for path in FONT_CANDIDATES:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size=size)  # Pillow >= 10.1
    except TypeError:
        return ImageFont.load_default()


def fitted_font(draw, text: str, size: int, max_width: int):
    """Largest font <= size at which text fits within max_width."""
    while size > 10:
        font = load_font(size)
        if draw.textlength(text, font=font) <= max_width:
            return font
        size -= 2
    return load_font(size)


def make_sign(payload: str, ssid: str, password: str) -> Image.Image:
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M,
                       box_size=24, border=4)
    qr.add_data(payload)
    qr.make(fit=True)
    code = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    q = code.size[0]

    # (text, y offset below the code, preferred font size)
    lines = [
        ("Scan to connect to the robot", 0, 64),
        (f"Wi-Fi: {ssid}", 95, 44),
        (f"Password: {password}", 155, 44),
        ("If the controls don't open automatically, open any web page", 240, 32),
    ]
    sign = Image.new("RGB", (SIGN_WIDTH, q + 360), "white")
    sign.paste(code, ((SIGN_WIDTH - q) // 2, 0))
    draw = ImageDraw.Draw(sign)
    for text, dy, size in lines:
        font = fitted_font(draw, text, size, SIGN_WIDTH - 2 * MARGIN)
        x = (SIGN_WIDTH - draw.textlength(text, font=font)) / 2
        draw.text((x, q + dy), text, fill="black", font=font)
    return sign


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Generate Wi-Fi QR codes (PNG sign + SVG) for BalanceBots.")
    ap.add_argument("robots", nargs="+", type=parse_suffix, metavar="SUFFIX",
                    help="MAC suffix from the serial log (e.g. 5302cc) or full SSID")
    ap.add_argument("-o", "--out", type=Path, default=DEFAULT_OUT,
                    help="output directory (default: tools/qr-codes/)")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    for suffix in args.robots:
        ssid = SSID_PREFIX + suffix
        password = PASSWORD_PREFIX + suffix
        payload = wifi_payload(ssid, password)

        png = args.out / f"{ssid}-wifi-qr.png"
        svg = args.out / f"{ssid}-wifi-qr.svg"
        make_sign(payload, ssid, password).save(png, dpi=(300, 300))
        qrcode.make(payload, image_factory=qrcode.image.svg.SvgPathImage).save(str(svg))
        print(f"{ssid}: {png}, {svg}")


if __name__ == "__main__":
    main()
