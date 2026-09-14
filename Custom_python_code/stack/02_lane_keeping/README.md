Classic lane keeping (no neural net).

Legal inputs: front RGB camera, speed.
Illegal as a driving input: map waypoints. Those are printed as ground truth only.

Run (CARLA must be open):

    python stack/02_lane_keeping/lane_keep.py
