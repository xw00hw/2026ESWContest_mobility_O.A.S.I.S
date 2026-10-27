"""v9 reference user: torso geometry from a recording of the Pi RTMPose viewer (pose-rpi.webm, 10/28 final session).

The viewer draws the runtime's COCO keypoints as green dots and the skeleton as yellow lines on the mono camera
image, scaled uniformly into its window. Per frame the shoulder-shoulder-hip-hip quad (all four sides drawn) is
found; shoulder width is taken as a fraction of the image width and shoulder-to-hip height as a fraction of the
image height, which is what oasis_rt.body_scale compares against (resolution independent).

  ffmpeg -t 61 -i pose-rpi.webm -vf fps=5 frames/f%04d.png
  python sim/v3/reference_body_from_video.py frames --out results/v9/reference_user.json

Frames with colour noise (decode corruption: the camera is monochrome) or without a quad are skipped.
"""
import argparse, itertools, json, os
import cv2
import numpy as np

REGION = (5, 1174, 60, 878)        # image area inside the 1179x882 screencast (x0, x1, y0, y1), from averaged frames


def masks(im):
    b, g, r = im[..., 0], im[..., 1], im[..., 2]
    green = (g > 150) & (g - r > 60) & (g - b > 60)
    yellow = (r > 170) & (g > 140) & (b < 150) & (r - b > 70)
    return green, yellow


def torso_quad(im, y0):
    green, yellow = masks(im)
    green = green.astype(np.uint8); green[:y0] = 0
    yellow = cv2.dilate(yellow.astype(np.uint8), np.ones((5, 5), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats(green)
    dots = [np.array(cen[i]) for i in range(1, n) if 12 <= st[i, cv2.CC_STAT_AREA] <= 400]
    N = len(dots); E = np.zeros((N, N))
    for i, j in itertools.combinations(range(N), 2):          # fraction of a straight segment drawn in yellow
        p, q = dots[i], dots[j]; L = np.linalg.norm(q - p)
        if L < 30: continue
        ts = np.linspace(12 / L, 1 - 12 / L, 25); pts = (p[None] + ts[:, None] * (q - p)[None]).round().astype(int)
        E[i, j] = E[j, i] = yellow[pts[:, 1], pts[:, 0]].mean()
    A = E >= 0.8; best = None
    for i, j in zip(*np.where(np.triu(A))):
        a, b = (i, j) if dots[i][0] < dots[j][0] else (j, i)
        for k in np.where(A[a])[0]:
            for l in np.where(A[b])[0]:
                if k == l or k in (a, b) or l in (a, b) or not A[k, l]: continue
                if not (dots[k][0] < dots[l][0] and max(dots[a][1], dots[b][1]) < min(dots[k][1], dots[l][1])): continue
                s = E[a, b] + E[k, l] + E[a, k] + E[b, l]
                if best is None or s > best[0]: best = (s, a, b, k, l)
    return None if best is None else [dots[x] for x in best[1:]]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('frames'); ap.add_argument('--out', required=True)
    ap.add_argument('--aspect', type=float, nargs='+', default=[800 / 1280, 480 / 640],
                    help='camera H/W candidates for the small vertical part of the shoulder span')
    a = ap.parse_args(); x0, x1, y0, y1 = REGION; W, H = x1 - x0, y1 - y0
    dx, dy, th, n = [], [], [], 0
    for f in sorted(os.listdir(a.frames)):
        im = cv2.imread(os.path.join(a.frames, f)); n += 1
        crop = im[y0:y1, x0:x1].astype(np.int16); g, y = masks(crop)
        if float((((crop.max(2) - crop.min(2)) > 40) & ~(g | y)).mean()) > 0.02: continue      # decode corruption
        q = torso_quad(im.astype(np.int16), y0)
        if q is None: continue
        sl, sr, hl, hr = q; sh, hp = (sl + sr) / 2, (hl + hr) / 2
        if (sr[0] - sl[0]) / W < 0.15 or (hp[1] - sh[1]) / H < 0.2: continue                    # not a seated torso
        dx.append((sr[0] - sl[0]) / W); dy.append((sr[1] - sl[1]) / H); th.append((hp[1] - sh[1]) / H)
    dx, dy, th = map(np.array, (dx, dy, th))
    by = {f'H/W {r:.3f}': float(np.median(np.hypot(dx, dy * r))) for r in a.aspect}
    w = float(np.mean(list(by.values()))); sw = np.hypot(dx, dy * a.aspect[0])
    out = dict(schema='oasis.v9.reference_user.v1', frames=dict(used=len(th), sampled=n),
               shoulder_width_frac=round(w, 4), torso_height_frac=round(float(np.median(th)), 4),
               shoulder_width_frac_by_aspect={k: round(v, 4) for k, v in by.items()},
               spread=dict(shoulder_cv=round(float(sw.std() / sw.mean()), 4), torso_cv=round(float(th.std() / th.mean()), 4)))
    json.dump(out, open(a.out, 'w'), indent=1); print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
