# extra_push.py
import os
import subprocess
import time
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

def _auto_push(source, target, env):
    # only run if there are changes
    project_dir = env['PROJECT_DIR']
    # stage any edits
    subprocess.run(["git", "add", "."], cwd=project_dir)
    # commit with timestamp (ignore if nothing to commit)
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    subprocess.run(
        ["git", "commit", "-m", f"Auto-build: {timestamp}"],
        cwd=project_dir,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    # push to your main branch
    subprocess.run(["git", "push", "origin", "main"], cwd=project_dir)

# run _auto_push after *both* build and upload
env.AddPostAction("buildprog",   _auto_push)
env.AddPostAction("upload",      _auto_push)
