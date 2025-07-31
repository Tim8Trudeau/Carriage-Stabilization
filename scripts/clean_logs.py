# clean_logs.py
import os
import glob

log_dir = "logs"
deleted = 0

for file in glob.glob(os.path.join(log_dir, "*.log")):
    try:
        os.remove(file)
        print(f"Deleted: {file}")
        deleted += 1
    except Exception as e:
        print(f"Failed to delete {file}: {e}")

print(f"{deleted} log files deleted.")
