import os


class _FallbackLogger:
    """Minimal logger with rospy-compatible methods."""

    @staticmethod
    def loginfo(msg):
        print(f"[INFO] {msg}")

    @staticmethod
    def logwarn(msg):
        print(f"[WARN] {msg}")

    @staticmethod
    def logerr(msg):
        print(f"[ERROR] {msg}")


def get_logger():
    """Return rospy if available, otherwise a lightweight console logger."""
    try:
        import rospy

        return rospy
    except Exception:
        return _FallbackLogger


def get_package_path(package_name="on_track_sys_id", package_path=None):
    """
    Resolve package root.

    Priority:
    1) Explicit package_path argument.
    2) ROS package lookup via rospkg.
    3) Local fallback based on this file location.
    """
    if package_path:
        return os.path.abspath(package_path)

    try:
        import rospkg

        return rospkg.RosPack().get_path(package_name)
    except Exception:
        # helpers/runtime.py -> helpers -> src -> package root
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
