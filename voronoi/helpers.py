import os


DEBUG_LEVEL = int(os.environ.get("DEBUG")) if "DEBUG" in os.environ else 0

def log(text: str, level: int = 1):
    if DEBUG_LEVEL >= level:
        print(text)


