# Reconstruction notes

- `BugNavigator` was reconstructed from the `Position_e-YRC.py` listing in the supplied PDF. Its key interfaces and behaviours are retained: `/scan`, `/odom`, `/cmd_vel`, waypoint tolerances, obstacle-distance threshold, and wall-following fallback.
- `FruitsTF` was reconstructed from the `task1b_fixed.py` listing. It retains RGB/depth subscriptions, HSV masking over a spatial region, depth ROI averaging, camera-to-base TF lookup, and per-detection TF broadcasting.
- `UR5WaypointNode` was reconstructed from the `UR5_e-YRC.py` listing. It retains the waypoint/state-machine pattern and TF-based pose feedback. Motion is disabled by default in this cleaned version.
- The source PDF was an image-based printout, not the original source tree. Some formatting and numeric OCR ambiguities required conservative cleanup. Verify every parameter and interface against the actual robot before use.
