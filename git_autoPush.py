Import("env")
print("▶ git_autoPush.py loaded!")
import os, subprocess
from datetime import datetime

def after_build(source, target, env):
    project_dir = env['PROJECT_DIR']
    os.chdir(project_dir)
    print("▶ extra_push: cwd =", project_dir)

    # 1) pull remote
    pull = subprocess.run(['git','pull','origin','main'],
                          capture_output=True, text=True, encoding="utf-8", errors="replace")
    print("▶ extra_push: git pull →", pull.stdout, pull.stderr)

    # 2) stage everything except RasPi/
    subprocess.run(['git','add','.'],           capture_output=True)
    subprocess.run(['git','reset','-q','RasPi/'], capture_output=True)

    # 3) if there’s something to commit, do it & push
    if subprocess.run(['git','diff','--cached','--quiet']).returncode != 0:
        msg = f"Auto‐update: {datetime.now():%Y-%m-%d %H:%M:%S}"
        print("▶ extra_push: committing:", msg)
        commit = subprocess.run(['git','commit','-m',msg],
                                capture_output=True, text=True, encoding="utf-8", errors="replace")
        print("▶ extra_push: git commit →", commit.stdout, commit.stderr)
        push = subprocess.run(['git','push','origin','main'],
                              capture_output=True, text=True, encoding="utf-8", errors="replace")
        print("▶ extra_push: git push →", push.stdout, push.stderr)
    else:
        print("▶ extra_push: nothing to commit")

# now hook that same function after linking the ELF (both build & upload)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", after_build)
env.AddPostAction("upload",                      after_build)
