import os

import rospkg


PackageNotFoundError = rospkg.ResourceNotFound


def get_package_share_directory(package_name):
    return rospkg.RosPack().get_path(package_name)


def get_package_prefix(package_name):
    path = get_package_share_directory(package_name)
    marker = os.sep + 'share' + os.sep + package_name
    return path.split(marker, 1)[0] if marker in path else os.path.dirname(path)

__all__ = [
    'PackageNotFoundError', 'get_package_prefix',
    'get_package_share_directory',
]
