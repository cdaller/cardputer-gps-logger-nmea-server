# PlatformIO post-build script:
#  dist/GPS-NMEA-Logger.bin       app image -> copy to SD card and install with the Launcher
#  dist/GPS-NMEA-Logger-full.bin  merged image (bootloader + partitions + app) for flashing at 0x0
import os
import shutil

Import("env")  # noqa: F821


def make_dist(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    dist = os.path.join(project_dir, "dist")
    os.makedirs(dist, exist_ok=True)

    app_bin = os.path.join(build_dir, "firmware.bin")
    shutil.copyfile(app_bin, os.path.join(dist, "GPS-NMEA-Logger.bin"))

    images = []
    for offset, path in env.get("FLASH_EXTRA_IMAGES", []):
        images += [offset, env.subst(path)]
    images += [env.subst("$ESP32_APP_OFFSET"), app_bin]

    cmd = " ".join([
        '"$PYTHONEXE"', '"$OBJCOPY"',
        "--chip", env.BoardConfig().get("build.mcu"),
        "merge_bin", "-o", '"%s"' % os.path.join(dist, "GPS-NMEA-Logger-full.bin"),
        "--flash_mode", "dio", "--flash_size", "8MB",
    ] + ['"%s"' % i for i in images])
    env.Execute(env.subst(cmd))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", make_dist)  # noqa: F821
