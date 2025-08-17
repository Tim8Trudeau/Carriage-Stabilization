# clean_logs.py
#!/usr/bin/env python3
import os, glob, sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
log_dir = os.path.join(REPO_ROOT, "logs")

deleted = 0
for file in glob.glob(os.path.join(log_dir, "*.log")):
    try:
        os.remove(file)
        print(f"Deleted: {os.path.relpath(file, REPO_ROOT)}")
        deleted += 1
    except Exception as e:
        print(f"Failed to delete {file}: {e}", file=sys.stderr)

print(f"{deleted} log file(s) deleted.")
