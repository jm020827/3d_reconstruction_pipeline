import os


def is_omniverse_path(path: str) -> bool:
    return isinstance(path, str) and path.startswith("omniverse://")


def path_exists(path: str) -> bool:
    # os.path.exists does not work for Nucleus (omniverse://) URLs.
    if is_omniverse_path(path):
        return True
    return os.path.exists(path)
