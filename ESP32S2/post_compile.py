import shutil
import os
Import("env")


def post_build_firmware(source, target, env):
    v = ''.join(c for c in env.GetProjectOption("firmware_version") if c.isdigit() or c in ['.'])
    board = env.GetProjectOption("board")
    bin = target[0].get_abspath()
    dest = os.path.join(os.path.pardir, os.path.pardir, env["PROJECT_DIR"], board + '-' + v + '.bin')
    print(bin + ' ->\n' + dest)
    shutil.copy(bin, dest)

    elf = source[0].get_abspath()
    dest = os.path.join(os.path.pardir, os.path.pardir, env["PROJECT_DIR"], board + '-' + v + '.elf')
    print(elf + ' ->\n' + dest)
    shutil.copy(elf, dest)

env.AddPostAction("$BUILD_DIR/firmware.bin", post_build_firmware)


def post_build_littlefs(source, target, env):
    v = ''.join(c for c in env.GetProjectOption("firmware_version") if c.isdigit() or c in ['.'])
    board = env.GetProjectOption("board")
    bin = target[0].get_abspath()
    dest = os.path.join(os.path.pardir, os.path.pardir, env["PROJECT_DIR"], board + '-' + v + '-littlefs.bin')
    print(bin + ' ->\n' + dest)
    shutil.copy(bin, dest)

env.AddPostAction("$BUILD_DIR/littlefs.bin", post_build_littlefs)


def post_build_bootloader(source, target, env):
    v = ''.join(c for c in env.GetProjectOption("firmware_version") if c.isdigit() or c in ['.'])
    board = env.GetProjectOption("board")
    bin = target[0].get_abspath()
    dest = os.path.join(os.path.pardir, os.path.pardir, env["PROJECT_DIR"], board + '-' + v + '-bootloader.bin')
    print(bin + ' ->\n' + dest)
    shutil.copy(bin, dest)

env.AddPostAction("$BUILD_DIR/bootloader.bin", post_build_bootloader)


def post_build_ota_data_initial(source, target, env):
    v = ''.join(c for c in env.GetProjectOption("firmware_version") if c.isdigit() or c in ['.'])
    board = env.GetProjectOption("board")
    bin = target[0].get_abspath()
    dest = os.path.join(os.path.pardir, os.path.pardir, env["PROJECT_DIR"], board + '-' + v + '-ota_data_initial.bin')
    print(bin + ' ->\n' + dest)
    shutil.copy(bin, dest)

env.AddPostAction("$BUILD_DIR/ota_data_initial.bin", post_build_ota_data_initial)


def post_build_partitions(source, target, env):
    v = ''.join(c for c in env.GetProjectOption("firmware_version") if c.isdigit() or c in ['.'])
    board = env.GetProjectOption("board")
    bin = target[0].get_abspath()
    dest = os.path.join(os.path.pardir, os.path.pardir, env["PROJECT_DIR"], board + '-' + v + '-partitions.bin')
    print(bin + ' ->\n' + dest)
    shutil.copy(bin, dest)

env.AddPostAction("$BUILD_DIR/partitions.bin", post_build_partitions)
