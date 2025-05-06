# git_autoPush.py

print("▶ extra_push.py loaded")

import os
import subprocess
from datetime import datetime
from SCons.Script import Import

# your existing function
def after_upload(source, target, env):
    project_dir = env['PROJECT_DIR']
    os.chdir(project_dir)
    print("▶ extra_push: cwd =", project_dir)

    # 1) pull any remote commits
    pull = subprocess.run(
        ['git', 'pull', 'origin', 'main'],
        capture_output=True, text=True
    )
    print("▶ extra_push: git pull stdout:\n", pull.stdout)
    if pull.stderr:
        print("▶ extra_push: git pull stderr:\n", pull.stderr)

    # 2) stage everything except RasPi/
    subprocess.run(['git', 'add', '.'], check=False)
    subprocess.run(['git', 'reset', '-q', 'RasPi/'], check=False)

    # 3) if there’s anything staged, commit & push
    diff = subprocess.run(['git', 'diff', '--cached', '--quiet'])
    if diff.returncode != 0:
        msg = f"Auto‐update Arduino: {datetime.now():%Y-%m-%d %H:%M:%S}"
        print("▶ extra_push: committing with message:", msg)
        commit = subprocess.run(
            ['git', 'commit', '-m', msg],
            capture_output=True, text=True
        )
        print("▶ extra_push: git commit stdout:\n", commit.stdout)
        if commit.stderr:
            print("▶ extra_push: git commit stderr:\n", commit.stderr)

        push = subprocess.run(
            ['git', 'push', 'origin', 'main'],
            capture_output=True, text=True
        )
        print("▶ extra_push: git push stdout:\n", push.stdout)
        if push.stderr:
            print("▶ extra_push: git push stderr:\n", push.stderr)
    else:
        print("▶ extra_push: nothing to commit")

# register with PlatformIO
Import("env")
env.AddPostAction("upload", after_upload)
