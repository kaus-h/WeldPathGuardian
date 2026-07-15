#!/usr/bin/env python3
"""Generate a compact WeldPath Guardian demo GIF."""

import math
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover - utility script guard
    raise SystemExit("Pillow is required: python3 -m pip install pillow") from exc


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "media" / "weldpath_guardian_demo.gif"
WIDTH = 720
HEIGHT = 405
FPS = 12
SECONDS = 20
FRAMES = FPS * SECONDS


def clean_point(t):
    x = t * 1.2
    y = 0.08 * math.sin(t * 2.2 * math.pi)
    z = 0.03 * math.cos(t * math.pi)
    return x, y, z


def project(point):
    x, y, z = point
    px = 72 + x * 470
    py = 236 - y * 900 - z * 360
    return int(px), int(py)


def noise(index, frame, scale):
    return math.sin(index * 12.9898 + frame * 0.31) * math.cos(index * 4.1414 + frame * 0.17) * scale


def phase(frame):
    second = frame / FPS
    if second < 4:
        return "VALIDATING", "clean seam", 0.0, 0.0, False, False
    if second < 8:
        return "EXECUTING", "gaussian noise", 0.008, 0.04, False, False
    if second < 12:
        return "FAULTED", "missing segment", 0.006, 0.0, True, False
    if second < 16:
        return "PAUSED", "low confidence", 0.006, 0.0, False, True
    return "EXECUTING", "valid data recovered", 0.004, 0.0, False, False


def draw_arrow(draw, start, end, fill):
    draw.line([start, end], fill=fill, width=3)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    for delta in (2.45, -2.45):
        head = (end[0] + 10 * math.cos(angle + delta), end[1] + 10 * math.sin(angle + delta))
        draw.line([end, head], fill=fill, width=3)


def make_frame(frame, font, small_font):
    state, scenario, noise_scale, dropout, gap, low_confidence = phase(frame)
    image = Image.new("RGB", (WIDTH, HEIGHT), "#101820")
    draw = ImageDraw.Draw(image)

    draw.rectangle([0, 0, WIDTH, 48], fill="#182430")
    draw.text((24, 14), "WeldPath Guardian", fill="#F4F7FB", font=font)
    draw.text((518, 14), f"state: {state}", fill="#F4F7FB", font=small_font)

    draw.line([(60, 305), (640, 305)], fill="#344556", width=1)
    draw.line([(60, 80), (60, 305)], fill="#344556", width=1)
    draw.text((62, 316), "perception -> planning -> execution -> monitoring", fill="#9FB0C1", font=small_font)

    clean = [clean_point(i / 63) for i in range(64)]
    raw = []
    valid = []
    for index, point in enumerate(clean):
        t = index / 63
        point_noise = (
            point[0] + noise(index, frame, noise_scale * 0.30),
            point[1] + noise(index + 7, frame, noise_scale),
            point[2] + noise(index + 19, frame, noise_scale * 0.50),
        )
        missing = gap and 0.42 < t < 0.58
        dropped = dropout > 0 and abs(noise(index + 29, frame, 1.0)) < dropout
        confidence_bad = low_confidence and 0.18 < t < 0.82
        raw.append(point_noise)
        valid.append(not (missing or dropped or confidence_bad))

    clean_pixels = [project(point) for point in clean]
    draw.line(clean_pixels, fill="#2E4658", width=2)

    fitted = [raw[index] for index, is_valid in enumerate(valid) if is_valid]
    if len(fitted) >= 4 and state != "FAULTED":
        fitted_pixels = [project(point) for point in fitted]
        draw.line(fitted_pixels, fill="#4AA3FF", width=4)
        for step in range(6, len(fitted) - 1, 9):
            draw_arrow(draw, project(fitted[step]), project(fitted[step + 1]), "#FFD247")

    for point, is_valid in zip(raw, valid):
        px, py = project(point)
        if is_valid:
            draw.ellipse([px - 4, py - 4, px + 4, py + 4], fill="#56D364")
        else:
            draw.line([(px - 6, py - 6), (px + 6, py + 6)], fill="#FF4D4D", width=3)
            draw.line([(px - 6, py + 6), (px + 6, py - 6)], fill="#FF4D4D", width=3)

    panel_x = 520
    draw.rectangle([panel_x, 72, 690, 240], fill="#17222D", outline="#2E4050")
    draw.text((panel_x + 14, 90), scenario, fill="#F4F7FB", font=small_font)
    draw.text((panel_x + 14, 120), "green: raw observations", fill="#56D364", font=small_font)
    draw.text((panel_x + 14, 144), "blue: fitted path", fill="#4AA3FF", font=small_font)
    draw.text((panel_x + 14, 168), "yellow: tool poses", fill="#FFD247", font=small_font)
    draw.text((panel_x + 14, 192), "red: rejected/fault", fill="#FF6B6B", font=small_font)

    progress = (frame % (FPS * 4)) / float(FPS * 4)
    if state == "FAULTED":
        progress = 0.0
    if state == "PAUSED":
        progress = 0.55
    draw.rectangle([70, 352, 650, 370], outline="#4B5E70")
    draw.rectangle([72, 354, 72 + int(574 * progress), 368], fill="#FFD247")
    draw.text((70, 376), "soft real-time pipeline with rejection, pause, and recovery paths", fill="#B8C7D6", font=small_font)

    return image


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    font = ImageFont.load_default()
    small_font = ImageFont.load_default()
    frames = [make_frame(frame, font, small_font) for frame in range(FRAMES)]
    frames[0].save(
        OUTPUT,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / FPS),
        loop=0,
        optimize=True,
    )
    print(OUTPUT)


if __name__ == "__main__":
    main()
