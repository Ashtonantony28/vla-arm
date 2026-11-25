"""Demo video of the MuJoCo twin for the README (simulation, scripted expert).

    MUJOCO_GL=egl python -m sim.demo_video [--out docs/media] [--seeds 3,5,8]

Rolls out the privileged-state scripted expert (sim/expert/scripted.py, the demonstrator the
policies are trained on) on pick_place, stack and sort, and renders each step from a slowly orbiting
free camera at 2x supersampling, with the wrist camera inset and the task instruction. Writes
arm-demo.{mp4,gif} and one still per task. Every frame is labelled as simulation.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from sim.envs.tabletop import CONTROL_HZ, TabletopEnv  # noqa: E402
from sim.expert.scripted import ScriptedExpert  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
W, H, SS = 1280, 720, 2
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def overlay(img, task, instruction, wrist, t, ok):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, 0, W, 92], fill=(13, 17, 23, 200))
    d.text((28, 14), f"vla-arm  ·  {task.replace('_', ' ')}", font=font(FONT_B, 30), fill=(230, 237, 243))
    d.text((28, 56), f"“{instruction}”", font=font(FONT, 22), fill=(173, 186, 199))
    d.rectangle([0, H - 40, W, H], fill=(13, 17, 23, 200))
    d.text((28, H - 32), "SIMULATION  ·  MuJoCo twin generated from spec/kinematics.yaml  ·  scripted expert demonstrations "
           "(the data the policies are trained on)", font=font(FONT, 15), fill=(139, 148, 158))
    wi = Image.fromarray(wrist).resize((320, 240), Image.LANCZOS)
    x0, y0 = W - 320 - 24, H - 240 - 56
    d.rectangle([x0 - 3, y0 - 28, x0 + 323, y0 + 243], fill=(13, 17, 23, 220))
    d.text((x0 + 4, y0 - 24), "wrist camera", font=font(FONT, 16), fill=(173, 186, 199))
    im.paste(wi, (x0, y0))
    d.text((W - 24, 108), f"t = {t:4.1f} s", font=font(FONT, 20), fill=(230, 237, 243), anchor="ra")
    if ok:
        d.rounded_rectangle([W - 190, 140, W - 24, 176], 8, fill=(35, 134, 54, 230))
        d.text((W - 107, 158), "SUCCESS", font=font(FONT_B, 20), fill=(255, 255, 255), anchor="mm")
    return np.asarray(im)


def episode(task, seed, frames_dir, f0, az0):
    env = TabletopEnv(task, image_size=96, render_images=False)
    m = env.model
    m.vis.global_.offwidth, m.vis.global_.offheight = W * SS, H * SS
    m.vis.quality.shadowsize = 8192
    obs, _ = env.reset(seed=seed)
    rmain = mujoco.Renderer(m, H * SS, W * SS)
    rwrist = mujoco.Renderer(m, 240, 320)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.22, 0.0, 0.11]
    cam.distance, cam.elevation = 0.92, -24
    expert = ScriptedExpert(env, seed=seed + 10_000, noise=True)
    f, ok, after, stills = f0, False, 0, None
    for k in range(env.max_steps):
        a = expert.act(obs).astype(np.float32)
        obs, _, term, trunc, info = env.step(a)
        ok = ok or info["is_success"]
        cam.azimuth = az0 + 0.12 * k
        rmain.update_scene(env.data, camera=cam)
        big = rmain.render()
        img = np.asarray(Image.fromarray(big).resize((W, H), Image.LANCZOS))
        rwrist.update_scene(env.data, camera="wrist")
        frame = overlay(img, task, env.instruction, rwrist.render(), (k + 1) / CONTROL_HZ, ok)
        Image.fromarray(frame).save(frames_dir / f"f{f:05d}.png")
        f += 1
        if ok:
            after += 1
            if after == 12:
                stills = frame
            if after > 30:
                break
        if trunc:
            break
    rmain.close()
    rwrist.close()
    return f, ok, stills


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "media"))
    ap.add_argument("--seeds", default="3,5,8")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp())
    f = 0
    for task, seed, az in zip(("pick_place", "stack", "sort"), map(int, a.seeds.split(",")), (128, 140, 152)):
        f, ok, still = episode(task, seed, tmp, f, az)
        print(task, seed, "success" if ok else "FAIL", f)
        if still is not None:
            Image.fromarray(still).save(out / f"arm-{task}.jpg", quality=90)
    mp4, gif = out / "arm-demo.mp4", out / "arm-demo.gif"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(CONTROL_HZ), "-i", str(tmp / "f%05d.png"), "-c:v", "libx264",
                    "-crf", "23", "-preset", "slow", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)
    # README GIF at 2x speed
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(mp4), "-vf",
                    "setpts=0.5*PTS,fps=8,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];"
                    "[b][p]paletteuse=dither=none:diff_mode=rectangle", str(gif)], check=True)
    for p in (mp4, gif):
        print(p, f"{p.stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
