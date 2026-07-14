# ============================================================
# B样条可视化（布局重排：控件与信息面板分离 + 基函数标签）
# ============================================================

import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Slider, Button
from matplotlib import font_manager as fm
import numpy as np
import os

# ------------------------------------------------------------
# 中文字体
# ------------------------------------------------------------
def force_chinese_font():
    candidates = [
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        "C:/Windows/Fonts/simhei.ttf",
        "C:/Windows/Fonts/msyh.ttc",
    ]
    font_path = None
    for fp in candidates:
        if os.path.exists(fp):
            font_path = fp
            break
    if font_path is None:
        raise RuntimeError("未找到中文字体！Ubuntu: apt install fonts-wqy-microhei")
    prop = fm.FontProperties(fname=font_path)
    font_name = prop.get_name()
    plt.rcParams["font.family"] = font_name
    plt.rcParams["font.sans-serif"] = [font_name]
    plt.rcParams["axes.unicode_minus"] = False

force_chinese_font()

# ------------------------------------------------------------
# Cox-de Boor 递推
# ------------------------------------------------------------
def cox_de_boor(i, k, u, knots):
    if k == 0:
        if i == len(knots) - 2:
            return 1.0 if knots[i] <= u <= knots[i + 1] else 0.0
        return 1.0 if knots[i] <= u < knots[i + 1] else 0.0
    d1 = knots[i + k] - knots[i]
    d2 = knots[i + k + 1] - knots[i + 1]
    t1 = (u - knots[i]) / d1 * cox_de_boor(i, k - 1, u, knots) if d1 > 1e-12 else 0.0
    t2 = (knots[i + k + 1] - u) / d2 * cox_de_boor(i + 1, k - 1, u, knots) if d2 > 1e-12 else 0.0
    return t1 + t2

def bspline_point(u, cps, knots, p):
    pt = np.zeros(2)
    for i in range(len(cps)):
        pt += cox_de_boor(i, p, u, knots) * np.array(cps[i])
    return pt

# ------------------------------------------------------------
# 全局状态
# ------------------------------------------------------------
NUM_CP = 6
P_DEG = 3
cps = [[float(i), float(np.random.randint(0, 5))] for i in range(NUM_CP)]
drag = {"active": False, "idx": -1}
u_norm = 0.5

fig = plt.figure(figsize=(16, 10))

# 左侧绘图网格
gs = fig.add_gridspec(3, 2, left=0.04, right=0.64, top=0.93, bottom=0.12,
                      wspace=0.30, hspace=0.52)
axes = [fig.add_subplot(gs[i // 2, i % 2]) for i in range(6)]

# 底部标签区
ax_legend = fig.add_axes([0.04, 0.01, 0.60, 0.09]); ax_legend.axis("off")

# ---- 右侧面板：控件集中顶部，信息面板在下方 ----
RX, RW = 0.69, 0.28
ax_ncp = fig.add_axes([RX, 0.90, RW, 0.032])
ax_slider = fig.add_axes([RX, 0.83, RW, 0.032])
ax_knots = fig.add_axes([RX, 0.76, RW, 0.032])
ax_btn = fig.add_axes([RX, 0.69, 0.12, 0.032])
# 信息面板：明确在控件下方，顶部 0.63 < 按钮底 0.69
ax_info = fig.add_axes([RX, 0.05, RW, 0.58]); ax_info.axis("off")

# 手动标签（避免 TextBox 自带左标签溢出）
fig.text(RX, 0.945, "控制点数量 N", fontsize=9.5, fontweight="bold")
fig.text(RX, 0.875, "阶数 p (0~4)", fontsize=9.5, fontweight="bold")
fig.text(RX, 0.805, "节点向量 (逗号分隔)", fontsize=9.5, fontweight="bold")

tb_ncp = TextBox(ax_ncp, "", initial=str(NUM_CP))
slider_p = Slider(ax_slider, "", 0, 4, valinit=P_DEG, valstep=1)
tb_knots = TextBox(ax_knots, "", initial="")
btn = Button(ax_btn, "更新")

# ------------------------------------------------------------
# 信息面板（紧凑，独立区间，无覆盖）
# ------------------------------------------------------------
def draw_info(n, p, m, knots, u, u_norm, active_s, active_e, cu):
    ax_info.clear()
    ax_info.axis("off")
    lines = [
        "====== B样条核心公式 ======",
        r"$m = n + p + 1$",
        f"n={n}, p={p}, m={m}",
        f"验算: {m}={n}+{p}+1  {'OK' if m == n + p + 1 else 'FAIL'}",
        f"u(归一化)={u_norm:.2f}  真实u={u:.3f}",
        f"C(u)=({cu[0]:.2f}, {cu[1]:.2f})",
        f"活跃控制点: P{active_s}~P{active_e} (共{p+1}个)",
        f"控制点={n+1}  节点={m+1}",
        f"段数={n-p+1}  域u∈[{knots[p]:.1f},{knots[n+1]:.1f}]",
    ]
    n_lines = len(lines)
    top, step = 0.98, 0.92 / max(n_lines, 1)
    for i, line in enumerate(lines):
        y = top - i * step
        is_formula = ("公式" in line) or ("验算" in line) or ("u(归一" in line) or ("活跃" in line)
        ax_info.text(0.02, y, line, fontsize=8.8, va="top",
                     transform=ax_info.transAxes,
                     fontweight="bold" if is_formula else "normal",
                     color="#b30000" if "公式" in line else "#222222")

# ------------------------------------------------------------
# 绘图核心
# ------------------------------------------------------------
def get_knots(n, p):
    knots = [float(x.strip()) for x in tb_knots.text.split(",") if x.strip()]
    if len(knots) != n + p + 2:
        knots = list(np.linspace(0, n + p + 1, n + p + 2))
        tb_knots.set_val(", ".join(f"{x:.1f}" for x in knots))
    return knots

markers = {}

def draw(_=None):
    global markers
    n = len(cps) - 1
    p = int(slider_p.val)
    knots = get_knots(n, p)

    umin, umax = knots[p], knots[n + 1]
    u = umin + u_norm * (umax - umin)

    k = p
    for ii in range(p, n + 1):
        if knots[ii] <= u < knots[ii + 1]:
            k = ii
            break
    else:
        if u >= knots[n + 1]:
            k = n
    active_s = max(0, k - p)
    active_e = min(n, k)
    active_set = set(range(active_s, active_e + 1))

    u_samples = np.linspace(umin, umax, 500)
    colors = plt.cm.tab10.colors
    ks = list(range(p, -1, -1))

    for key in list(markers.keys()):
        if markers[key] is not None:
            try:
                markers[key].remove()
            except Exception:
                pass
    markers.clear()

    # ---- 基函数子图（含 N_{i,k} 标签，防重叠）----
    for i, ax in enumerate(axes):
        ax.clear()
        if i < len(ks):
            kk = ks[i]
            placed = []
            for j in range(len(knots) - 1 - kk):
                vals = [cox_de_boor(j, kk, ui, knots) for ui in u_samples]
                ax.plot(u_samples, vals, color=colors[j % 10], lw=1.5)
                peak = np.max(vals)
                if peak > 0.08:
                    idx = int(np.argmax(vals))
                    xu = u_samples[idx]
                    yu = peak + 0.05
                    for (px, py) in placed:
                        if abs(xu - px) < 0.5 and abs(yu - py) < 0.14:
                            yu = py + 0.14
                    placed.append((xu, yu))
                    ax.text(xu, yu, rf"$N_{{{j},{kk}}}$", fontsize=7.5,
                            color=colors[j % 10], ha="center", va="bottom")
            ax.axvline(u, color="red", ls="--", lw=1.0, alpha=0.7)
            for j in range(len(knots) - 1 - kk):
                val = cox_de_boor(j, kk, u, knots)
                if val > 1e-4:
                    markers[f"base_{i}_{j}"] = ax.plot(u, val, "ro", ms=4)[0]
            for kk2 in knots:
                ax.axvline(kk2, color="gray", ls=":", lw=0.5, alpha=0.4)
            ax.set_title(rf"k = {kk} : $N_{{i,{kk}}}(u)$", fontsize=8)
            ax.set_xlabel("参数 u")
            ax.set_ylim(-0.1, 1.45)
            ax.grid(True, alpha=0.25)
        else:
            ax.axis("off")

    # ---- B样条曲线 ----
    curve_ax = axes[len(ks)]
    curve = np.array([bspline_point(ui, cps, knots, p) for ui in u_samples])
    curve_ax.plot(curve[:, 0], curve[:, 1], "b-", lw=2.5)
    cps_arr = np.array(cps)
    curve_ax.plot(cps_arr[:, 0], cps_arr[:, 1], "k--", lw=1, alpha=0.4)

    act_idx = [i for i in range(len(cps)) if i in active_set]
    inact_idx = [i for i in range(len(cps)) if i not in active_set]
    if act_idx:
        curve_ax.scatter(cps_arr[act_idx, 0], cps_arr[act_idx, 1],
                         c="gold", s=150, zorder=10, edgecolors="black", linewidths=1.5)
    if inact_idx:
        curve_ax.scatter(cps_arr[inact_idx, 0], cps_arr[inact_idx, 1],
                         facecolors="none", s=150, zorder=10,
                         edgecolors="gray", linewidths=1.2)
    cu = bspline_point(u, cps, knots, p)
    markers["cu"] = curve_ax.plot(cu[0], cu[1], "go", ms=10)[0]
    curve_ax.set_title(f"B样条曲线 (p={p})", fontsize=9.5)
    curve_ax.set_aspect("equal")
    curve_ax.grid(True, alpha=0.25)

    # ---- 底部标签区 ----
    ax_legend.clear()
    ax_legend.axis("off")
    ax_legend.text(0.02, 0.72,
                   "● 活跃控制点(黄)    ○ 非活跃控制点(空)    — B样条曲线(蓝)    ● C(u)当前点(绿)",
                   fontsize=10, va="center", fontweight="bold")
    ax_legend.text(0.02, 0.28,
                   "操作: ← / → 方向键调整参数 u (步长 0.05, 归一化)   |   鼠标拖拽红点移动控制点",
                   fontsize=9, va="center", color="#555555")

    draw_info(n, p, len(knots) - 1, knots, u, u_norm, active_s, active_e, cu)
    fig.canvas.draw_idle()

# ------------------------------------------------------------
# 键盘调 u
# ------------------------------------------------------------
def on_key(event):
    global u_norm
    if event.key == "left":
        u_norm = max(0.0, u_norm - 0.05)
        draw()
    elif event.key == "right":
        u_norm = min(1.0, u_norm + 0.05)
        draw()

fig.canvas.mpl_connect("key_press_event", on_key)

# ------------------------------------------------------------
# 拖拽控制点
# ------------------------------------------------------------
curve_ax = axes[1]

def on_press(e):
    if e.inaxes != curve_ax or e.xdata is None:
        return
    xs, ys = zip(*cps)
    d = np.hypot(np.array(xs) - e.xdata, np.array(ys) - e.ydata)
    idx = int(np.argmin(d))
    if d[idx] < 0.25:
        drag["active"], drag["idx"] = True, idx

def on_motion(e):
    global curve_ax
    curve_ax = axes[min(int(slider_p.val) + 1, 5)]
    if not drag["active"] or e.inaxes != curve_ax or e.xdata is None:
        return
    cps[drag["idx"]] = [e.xdata, e.ydata]
    draw()

def on_release(e):
    if drag["active"]:
        drag["active"], drag["idx"] = False, -1
        draw()

fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)

# ------------------------------------------------------------
# 回调
# ------------------------------------------------------------
def on_ncp(t):
    try:
        n = max(3, min(8, int(t)))
        global cps
        cps = [[float(i), float(np.random.randint(0, 5))] for i in range(n)]
        draw()
    except ValueError:
        pass

tb_ncp.on_submit(on_ncp)
slider_p.on_changed(draw)
tb_knots.on_submit(draw)
btn.on_clicked(draw)

draw()
plt.show()