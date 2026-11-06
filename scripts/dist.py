# Copies the merged (bootloader + partitions + app) image to dist/ for M5Burner and other
# offset-0x0 flashers, plus the app-only image for M5Launcher / OTA-style installers.
import os
import shutil

Import("env")  # noqa: F821


def copy_dist(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    dist = os.path.join(project_dir, "dist")
    os.makedirs(dist, exist_ok=True)
    pairs = [
        ("firmware.factory.bin", "cardputer-gps-nav-m5burner.bin"),
        ("firmware.bin", "cardputer-gps-nav-app.bin"),
    ]
    for src_name, dst_name in pairs:
        src = os.path.join(build_dir, src_name)
        if os.path.exists(src):
            shutil.copyfile(src, os.path.join(dist, dst_name))
            print("[dist] %s -> dist/%s" % (src_name, dst_name))
        else:
            print("[dist] missing %s" % src)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_dist)  # noqa: F821
